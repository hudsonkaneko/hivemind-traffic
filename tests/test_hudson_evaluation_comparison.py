from scripts.compare_hudson_evaluation import differences


def test_exact_and_tolerant_are_separate():
    assert differences({'progress':1.}, {'progress':1.})==[]
    changed=differences({'progress':1.}, {'progress':1.+1e-10})
    assert len(changed)==1 and changed[0]['within_tolerance']
    assert not differences({'progress':1.}, {'progress':1.01})[0]['within_tolerance']


def test_integer_counts_structure_and_missing_values_never_disappear():
    for r,a in [(True,1),([1,2],[1]),({'a':1},{}),(None,0),({'seed':410001},{'seed':410002})]:
        mismatch=differences(r,a)
        assert mismatch and any(not x['within_tolerance'] for x in mismatch)


def test_nonfinite_values_fail_even_when_both_are_infinite():
    assert not differences(float('inf'),float('inf'))[0]['within_tolerance']
