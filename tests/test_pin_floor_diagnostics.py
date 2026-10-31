"""Synthetic geometric counterexamples, not official scoring fixtures."""
from pathlib import Path
import json, subprocess, sys, tempfile, unittest

BINARY=Path(sys.argv.pop(1)).resolve()

class PinFloorDiagnostics(unittest.TestCase):
    def test_equal_total_does_not_mean_equal_resource_map(self):
        with tempfile.TemporaryDirectory(prefix='pin-floor-discrimination-') as temp:
            root=Path(temp)
            lef=''.join(f'LAYER metal{i} TYPE ROUTING ; DIRECTION VERTICAL ; WIDTH 0.1 ; END metal{i}\n' for i in range(1,21))
            lef+='MACRO cell SIZE 6 BY 6 ; PIN VDD PORT LAYER metal1 ; RECT 0 0.1 2.9 0.2 ; RECT 0 3.1 2.9 3.2 ; END END VDD\n'
            # SE has a real-track RECT and a clipped-empty RECT in the same
            # cell. CK has no real track but its next index overlaps VDD.
            lef+='PIN SE PORT LAYER metal1 ; RECT 0.1 0.4 0.3 0.5 ; RECT 2.928 0.4 3.188 0.5 ; END END SE\n'
            lef+='PIN CK PORT LAYER metal1 ; RECT 1.208 3.4 1.368 3.5 ; END END CK\n'
            lef+='OBS LAYER metal1 ; RECT 3.2 3.1 3.4 3.2 ; END END cell\n'
            definition='UNITS DISTANCE MICRONS 1000 ; DIEAREA ( 0 0 ) ( 6000 6000 ) ; GCELLGRID X 0 DO 3 STEP 3000 ; GCELLGRID Y 0 DO 3 STEP 3000 ; TRACKS X 0 DO 31 STEP 200 LAYER metal1 ; COMPONENTS 1 ; - i cell + PLACED ( 0 0 ) N ; END COMPONENTS PINS 0 ; END PINS NETS 1 ; - n ( i CK ) ; END NETS END DESIGN\n'
            (root/'in.lef').write_text(lef);(root/'in.def').write_text(definition)
            # One real net makes the design valid. Its Guide is entirely in
            # its own PIN cell, so it contributes zero dynamic demand.
            (root/'fixture.guide').write_text('n\n(\n0 3000 3000 6000 metal1\n)\n')
            (root/'expected.csv').write_text('layer,gcell_col,gcell_row,capacity,demand\nmetal1,0,0,15,16\n')
            command=[str(BINARY),str(root/'in.def'),str(root/'in.lef'),str(root/'fixture.guide'),str(root/'expected.csv'),str(root/'out.json')]
            p=subprocess.run(command,capture_output=True,text=True,timeout=30)
            self.assertEqual(p.returncode,0,p.stdout+p.stderr)
            data=json.loads((root/'out.json').read_text());models=data['models']
            self.assertTrue(data['diagnostic_only']);self.assertFalse(data['production_allowed'])
            self.assertEqual(data['raw_track_union_differences'],0)
            self.assertTrue(models['geometry_next_union']['observed_overflow_map_exact'])
            self.assertEqual(models['production_logical_additive']['overflow'],1)
            self.assertEqual(models['geometry_next_union']['overflow'],1)
            self.assertEqual(models['production_logical_additive']['extra'],1)
            self.assertEqual(models['production_logical_additive']['missing'],1)
            self.assertFalse(models['whole_shape_empty_next_union']['observed_overflow_map_exact'])
            self.assertEqual(models['geometry_drop_outside']['missing'],1)
            # This net is wholly PIN-exempt: fixed overflow cannot be repaired
            # by removing its already-zero Guide demand.
            for model in models.values():
                self.assertEqual(model['relaxation']['removable_Q_upper_bound'],0)
                self.assertEqual(model['relaxation']['removable_cells'],0)
                self.assertEqual(model['relaxation']['C_upper_bound'],model['C'])
            # Existing evidence is never overwritten by a rerun.
            before=(root/'out.json').read_bytes()
            p=subprocess.run(command,capture_output=True,text=True,timeout=30)
            self.assertNotEqual(p.returncode,0)
            self.assertEqual((root/'out.json').read_bytes(),before)
            # A real PIN endpoint exactly on a track distinguishes the joint
            # half-open/minimum-union model from the closed-interval probe.
            (root/'in.lef').write_text(lef.replace('2.9','2.8'))
            (root/'expected.csv').write_text('layer,gcell_col,gcell_row,capacity,demand\n')
            command[-1]=str(root/'endpoint.json')
            p=subprocess.run(command,capture_output=True,text=True,timeout=30)
            self.assertEqual(p.returncode,0,p.stdout+p.stderr)
            endpoint=json.loads((root/'endpoint.json').read_text());models=endpoint['models']
            self.assertEqual(models['geometry_next_union']['extra'],1)
            self.assertTrue(models['geometry_next_union_open_pins']['observed_overflow_map_exact'])
            self.assertTrue(models['geometry_next_union_open_all']['observed_overflow_map_exact'])
            # Both hypotheses fit the overflow map yet disagree on a
            # nonoverflowing OBS boundary. Sparse observations are insufficient.
            self.assertEqual(models['geometry_next_union_open_pins']['fixed_total']-models['geometry_next_union_open_all']['fixed_total'],1)
            self.assertEqual(endpoint['pair_disagreements'][1]['fixed_different_cells'],1)

if __name__=='__main__':unittest.main()
