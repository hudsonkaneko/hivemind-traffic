"""v15 Phase 3: the plain-PyTorch trainer for the learned Level 2 (behaviour cloning and DAgger refits).

Plan v15, Stages 2-3, approved November 26, 2026 ("Recommended size", relayed by the main Claude session).
CPU, one thread, deterministic (torch.use_deterministic_algorithms): the same data and fit seed give the same
weights. Nothing here simulates; the caller charges the fit's clock time to its ledger segment.

- **Data:** the level2.pt sample files the pool writes (traffic/v15_level2.Level2Driver.sample_batch): the
  124 non-privileged inputs, the rule Level 2's labels (lane, gap, offset), validity masks and weights.
- **Loss:** cross-entropy per choice, weighted by each sample's weight, over the samples where the choice
  applies (lane: not during a maneuver; gap: at an active yield entry; offset: always), summed over the
  three choices.
- **Split by seed:** the caller passes the training and held-back sample files (whole seed sets). The
  held-back loss and accuracy are reported for memorizing only; models are chosen by closed-loop driving.
- **Settings:** FIT (AdamW, 30 epochs, batch 1024); the fit seed sets the initial weights and the order. A fit
  takes 30 passes' worth of updates over at most `max_samples_per_epoch` samples (200,000): on more data (the
  later DAgger refits) the passes run on, over fresh permutations, for the same number of updates, so a refit's
  clock time stays near a cloning fit's (about 1.2 s per pass over 170,000 samples on one thread, measured on
  November 26 on synthetic data: about 35 s for 30 passes).
"""
from __future__ import annotations

import math
import time

FIT = dict(epochs=30, batch=1024, lr=1e-3, weight_decay=1e-4, threads=1, max_samples_per_epoch=200_000)
FIELDS = ('x', 'lane', 'gap', 'offset', 'lane_valid', 'gap_valid', 'weight')


def load(paths):
    """Concatenate level2.pt sample files (in the given order) into one dataset."""
    import torch
    parts = [torch.load(path, map_location='cpu', weights_only=False) for path in paths]
    if not parts:
        raise ValueError('No sample files')
    return {name: torch.cat([part[name] for part in parts]) for name in FIELDS}


def _loss(net, batch, reduction='mean'):
    import torch.nn.functional as F
    lane, gap, offset = net(batch['x'])
    w = batch['weight']
    out = {}
    for name, logits, valid in (('lane', lane, batch['lane_valid']), ('gap', gap, batch['gap_valid']),
                                ('offset', offset, None)):
        labels = batch[name].clamp(min=0)
        weight = w if valid is None else w * valid.float()
        ce = F.cross_entropy(logits, labels, reduction='none')
        total = weight.sum()
        out[name] = (ce * weight).sum() / total if float(total) > 0 else ce.sum() * 0.
        out[name + '_correct'] = ((logits.argmax(1) == labels).float() * (weight > 0).float()).sum()
        out[name + '_count'] = (weight > 0).float().sum()
    out['total'] = out['lane'] + out['gap'] + out['offset']
    return out


def evaluate(net, data, batch=8192):
    """Weighted loss and unweighted accuracy per choice on a dataset (no gradient)."""
    import torch
    sums = dict(lane=0., gap=0., offset=0.)
    weights = dict(lane=0., gap=0., offset=0.)
    correct = dict(lane=0., gap=0., offset=0.)
    count = dict(lane=0., gap=0., offset=0.)
    n = data['x'].shape[0]
    net.eval()
    with torch.no_grad():
        for start in range(0, n, batch):
            part = {k: v[start:start + batch] for k, v in data.items()}
            out = _loss(net, part)
            for name, valid in (('lane', part['lane_valid']), ('gap', part['gap_valid']), ('offset', None)):
                w = part['weight'] if valid is None else part['weight'] * valid.float()
                sums[name] += float(out[name]) * float(w.sum())
                weights[name] += float(w.sum())
                correct[name] += float(out[name + '_correct'])
                count[name] += float(out[name + '_count'])
    return {name: dict(loss=sums[name] / weights[name] if weights[name] else None,
                       accuracy=correct[name] / count[name] if count[name] else None, samples=int(count[name]))
            for name in sums}


def fit(train, *, seed, held_back=None, settings=FIT, log=None):
    """Train a Level2Net from scratch on `train`; returns (net, report). Deterministic for a given seed."""
    import torch
    from traffic import v15_level2
    started = time.perf_counter()
    torch.set_num_threads(int(settings['threads']))
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(int(seed))
    net = v15_level2.build_net()
    optimizer = torch.optim.AdamW(net.parameters(), lr=settings['lr'], weight_decay=settings['weight_decay'])
    import torch.nn.functional as F
    order = torch.Generator().manual_seed(int(seed) + 1)
    n, batch = int(train['x'].shape[0]), int(settings['batch'])
    if n == 0:
        raise ValueError('No training samples')
    per_epoch = math.ceil(min(n, int(settings['max_samples_per_epoch'])) / batch)
    steps = int(settings['epochs']) * per_epoch
    weights = (train['weight'] * train['lane_valid'].float(), train['weight'] * train['gap_valid'].float(), train['weight'])
    labels = (train['lane'].clamp(min=0), train['gap'].clamp(min=0), train['offset'])
    history, done, net = [], 0, net.train()
    while done < steps:
        permutation = torch.randperm(n, generator=order)
        x = train['x'][permutation]
        ys = [y[permutation] for y in labels]
        ws = [w[permutation] for w in weights]
        total, count = 0., 0
        for start in range(0, n, batch):
            if done >= steps:
                break
            stop = start + batch
            loss = 0.
            for logits, y, w in zip(net(x[start:stop]), ys, ws):
                part, weight = y[start:stop], w[start:stop]
                mass = weight.sum()
                if float(mass) > 0:
                    loss = loss + (F.cross_entropy(logits, part, reduction='none') * weight).sum() / mass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total += float(loss.detach())
            count += 1
            done += 1
            if count == per_epoch:
                history.append(round(total / count, 6))
                if log:
                    log(dict(epoch=len(history) - 1, loss=history[-1]))
                total, count = 0., 0
        if count:
            history.append(round(total / count, 6))
    net.eval()
    report = dict(seed=int(seed), samples=n, steps=steps, settings=dict(settings), epoch_loss=history,
                  train=evaluate(net, train), held_back=evaluate(net, held_back) if held_back is not None else None,
                  fit_seconds=time.perf_counter() - started)
    return net, report
