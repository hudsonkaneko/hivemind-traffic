from types import SimpleNamespace
import pytest
import traffic.runtime_metrics as metrics


class Process:
    def __init__(self): self.calls=0
    def memory_info(self):
        self.calls+=1
        return SimpleNamespace(rss=self.calls*100, private=self.calls*80)


def test_memory_sampling_is_low_frequency_and_bounded():
    process=Process(); sampler=metrics.MemorySampler(process=process, max_samples=3)
    for second in range(41): sampler.sample(second)
    report=sampler.summary()
    assert process.calls==5
    assert len(report['samples'])==3
    assert report['rss_delta_bytes']==400
    assert report['rss_peak_bytes']==500
    assert report['last_half_slope_bytes_per_second']==pytest.approx(10)


def test_final_forced_sample_and_invalid_times():
    sampler=metrics.MemorySampler(process=Process())
    sampler.sample(0); assert sampler.sample(1) is None
    assert sampler.sample(1,force=True)['rss_bytes']==200
    with pytest.raises(ValueError): sampler.sample(float('nan'))


class FakeGC:
    def __init__(self, enabled=True, frozen=0):
        self.enabled=enabled; self.frozen=frozen; self.callbacks=[]; self.unfreezes=0
    def isenabled(self): return self.enabled
    def enable(self): self.enabled=True
    def disable(self): self.enabled=False
    def get_freeze_count(self): return self.frozen
    def collect(self):
        for callback in self.callbacks: callback('start',dict(generation=2))
        for callback in self.callbacks: callback('stop',dict(generation=2))
    def freeze(self): self.frozen=10
    def unfreeze(self): self.frozen=0; self.unfreezes+=1


@pytest.mark.parametrize('mode',['normal','deferred','freeze-established'])
@pytest.mark.parametrize('enabled',[True,False])
def test_scoped_gc_restores_state_on_exception(monkeypatch,mode,enabled):
    gc=FakeGC(enabled); monkeypatch.setattr(metrics,'gc',gc)
    observer=lambda *args: None; gc.callbacks.append(observer)
    with pytest.raises(RuntimeError):
        with metrics.ScopedGC(mode) as profile:
            if mode=='deferred': assert not gc.enabled
            if mode=='freeze-established': assert gc.enabled and gc.frozen
            raise RuntimeError('fixture')
    assert gc.enabled==enabled and gc.callbacks==[observer] and gc.frozen==0
    assert profile.summary()['collection_count']==(0 if mode=='normal' else 1)


def test_preexisting_frozen_graph_is_preserved(monkeypatch):
    gc=FakeGC(frozen=123); monkeypatch.setattr(metrics,'gc',gc)
    with metrics.ScopedGC('freeze-established') as profile:
        assert profile.effective_mode=='normal'
    assert gc.frozen==123 and gc.unfreezes==0
