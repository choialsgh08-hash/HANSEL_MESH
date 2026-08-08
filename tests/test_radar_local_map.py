import math
import unittest

from monitor.radar_local_map import ProbabilisticRollingGrid


class ProbabilisticRollingGridTests(unittest.TestCase):
    def make_grid(self):
        return ProbabilisticRollingGrid(radius_m=0.50, resolution_m=0.05, occupied_threshold=0.75, decay_half_life_s=1.0, ttl_s=2.5, hit_probability=0.80, maximum_probability=0.97)

    def test_unknown_map_starts_without_invented_free_cells(self): self.assertEqual(self.make_grid().payload(1.0)["cells"], [])
    def test_confirmed_hit_is_occupied_and_preserves_uncertainty(self):
        grid=self.make_grid(); grid.add_hit(0.31,-0.04,0.8,"point",0.049,0.052,1.0); cell=grid.payload(1.0)["cells"][0]
        self.assertEqual((cell["forward_m"],cell["lateral_m"]),(0.30,-0.05)); self.assertAlmostEqual(cell["probability"],0.80,places=3); self.assertTrue(cell["occupied"]); self.assertEqual(cell["hits"],1); self.assertEqual(cell["range_uncertainty_m"],0.049)
    def test_one_half_life_decays_toward_unknown_without_becoming_free(self):
        grid=self.make_grid(); grid.add_hit(0.30,0.0,0.9,"point",0.05,0.05,1.0); cell=grid.payload(2.0)["cells"][0]
        self.assertAlmostEqual(cell["probability"],2.0/3.0,places=3); self.assertFalse(cell["occupied"]); self.assertGreater(cell["probability"],0.5); self.assertEqual(grid.diagnostics()["decayed_cell_count"],1)
    def test_cell_is_removed_at_exact_2500ms_ttl(self):
        grid=self.make_grid(); grid.add_hit(0.30,0.0,0.9,"point",0.05,0.05,1.0); self.assertEqual(grid.payload(3.5)["cells"],[]); self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"],1)
    def test_probability_and_hit_count_are_capped(self):
        grid=self.make_grid()
        for _ in range(300): grid.add_hit(0.30,0.0,0.9,"point",0.05,0.05,1.0)
        cell=grid.payload(1.0)["cells"][0]; self.assertAlmostEqual(cell["probability"],0.97,places=6); self.assertEqual(cell["hits"],255)
    def test_repeated_key_conservatively_merges_metadata_and_source(self):
        grid=self.make_grid(); grid.add_hit(0.30,0.0,0.9,"point",0.08,0.03,1.0); grid.add_hit(0.31,0.01,0.6,"heatmap",0.02,0.09,1.0); cell=grid.payload(1.0)["cells"][0]
        self.assertEqual(cell["confidence"],0.9); self.assertEqual(cell["range_uncertainty_m"],0.08); self.assertEqual(cell["lateral_uncertainty_m"],0.09); self.assertEqual(cell["source"],"mixed")
    def test_authorized_transform_rebins_without_duplicating_probability(self):
        grid=self.make_grid(); grid.add_hit(0.30,0.0,0.9,"point",0.05,0.05,1.0); grid.transform(-0.05,0.0,0.0,1.0); cell=grid.occupied_cells(1.0)[0]
        self.assertEqual(cell["forward_m"],0.25); self.assertAlmostEqual(cell["probability"],0.80,places=3)
    def test_transform_drops_rebinned_cell_outside_map_without_ttl_count(self):
        grid=self.make_grid(); grid.add_hit(0.48,0.0,0.9,"point",0.05,0.05,1.0); grid.transform(0.05,0.0,0.0,1.0); self.assertEqual(grid.payload(1.0)["cells"],[]); self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"],0)
    def test_transform_uses_old_to_new_rotation_then_translation(self):
        grid=self.make_grid(); grid.add_hit(0.30,0.10,0.9,"point",0.05,0.05,1.0); grid.transform(0.05,-0.02,math.radians(90.0),1.0); cell=grid.payload(1.0)["cells"][0]
        self.assertEqual((cell["forward_m"],cell["lateral_m"]),(-0.05,0.30))
    def test_transform_collision_conservatively_merges_without_summing(self):
        grid=self.make_grid(); grid.add_hit(0.05,-0.40,0.9,"point",0.08,0.03,1.0); grid.add_hit(0.05,-0.35,0.6,"heatmap",0.02,0.09,1.0); grid.transform(0.0,0.0,math.radians(15.0),1.0); cells=grid.payload(1.0)["cells"]
        self.assertEqual(len(cells),1); self.assertAlmostEqual(cells[0]["probability"],0.80,places=3); self.assertEqual(cells[0]["hits"],1); self.assertEqual(cells[0]["confidence"],0.9); self.assertEqual(cells[0]["range_uncertainty_m"],0.08); self.assertEqual(cells[0]["lateral_uncertainty_m"],0.09); self.assertEqual(cells[0]["source"],"mixed")
    def test_decay_and_ttl_counters_are_poll_idempotent(self):
        grid=self.make_grid(); grid.add_hit(0.30,0.0,0.9,"point",0.05,0.05,1.0); grid.payload(2.0); grid.payload(2.0); grid.occupied_cells(2.0); self.assertEqual(grid.diagnostics()["decayed_cell_count"],1); grid.payload(3.5); grid.payload(3.5); self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"],1)
    def test_occupied_cells_poll_alone_advances_decay_and_ttl(self):
        grid=self.make_grid(); grid.add_hit(0.30,0.0,0.9,"point",0.05,0.05,1.0); self.assertEqual(grid.occupied_cells(2.0),[]); self.assertEqual(grid.diagnostics()["decayed_cell_count"],1); self.assertEqual(grid.occupied_cells(3.5),[]); self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"],1)
    def test_clear_removes_cells_and_resets_every_counter(self):
        grid=self.make_grid(); grid.add_hit(0.30,0.0,0.9,"point",0.05,0.05,1.0); grid.payload(2.0); grid.payload(3.5); self.assertEqual(grid.diagnostics()["decayed_cell_count"],1); self.assertEqual(grid.diagnostics()["ttl_expired_cell_count"],1); grid.clear(); self.assertEqual(grid.payload(3.5)["cells"],[])
        self.assertEqual(grid.diagnostics(),{"active_cell_count":0,"occupied_cell_count":0,"decayed_cell_count":0,"ttl_expired_cell_count":0})
