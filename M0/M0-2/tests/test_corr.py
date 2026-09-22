import unittest
import tempfile
from pathlib import Path

from legacy_corr import calculate_correlation, load_csv_data


class TestCorrelation(unittest.TestCase):

    def test_positive_correlation(self):
        xs = [1, 2, 3]
        ys = [2, 4, 6]

        _, _, _, r = calculate_correlation(xs, ys)

        self.assertAlmostEqual(r, 1.0)

    def test_negative_correlation(self):
        xs = [1, 2, 3]
        ys = [6, 4, 2]

        _, _, _, r = calculate_correlation(xs, ys)

        self.assertAlmostEqual(r, -1.0)

    def test_zero_variance(self):
        xs = [5, 5, 5]
        ys = [1, 2, 3]

        with self.assertRaises(ValueError):
            calculate_correlation(xs, ys)

    def test_empty_data(self):
        xs = []
        ys = []

        with self.assertRaises(ValueError):
            calculate_correlation(xs, ys)

    def test_missing_column(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = Path(tmpdir) / "test.csv"

            csv_path.write_text(
                "sensor_a,wrong_column\n"
                "1,2\n"
                "2,4\n"
            )

            with self.assertRaises(ValueError):
                load_csv_data(csv_path, "sensor_a", "sensor_b")


if __name__ == "__main__":
    unittest.main()
