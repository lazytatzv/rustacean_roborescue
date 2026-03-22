import csv
from datetime import datetime
from pathlib import Path


class POIWriter:
    """Simple POI CSV writer matching RoboCup format (minimal)."""

    def __init__(
        self,
        out_dir: str = "docs/outputs",
        team_name: str = "Team",
        country: str = "Country",
    ):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.team_name = team_name
        self.country = country
        self.sequence = 0
        self.filepath = self._make_filepath()
        self._ensure_header()

    def _make_filepath(self):
        now = datetime.now()
        fname = f"RoboCup{now.year}-{self.team_name}-Mission-{now.strftime('%H-%M-%S')}-pois.csv"
        return self.out_dir / fname

    def _ensure_header(self):
        if not self.filepath.exists():
            with open(self.filepath, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(['"pois"'])
                w.writerow(['"1.3"'])
                w.writerow([f'"{self.team_name}"'])
                w.writerow([f'"{self.country}"'])
                w.writerow([datetime.now().strftime('"%Y-%m-%d"')])
                w.writerow([datetime.now().strftime('"%H:%M:%S"')])
                w.writerow(['"Prelim1"'])
                w.writerow(["detection,time,type,name,x,y,z,robot,mode"])

    def add_heat_detection(
        self,
        name: str = "0",
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        robot: str = "robot1",
        mode: str = "A",
    ):
        self.sequence += 1
        now = datetime.now().strftime("%H:%M:%S")
        row = [
            self.sequence,
            now,
            "heat_sig",
            name,
            f"{x:.3f}",
            f"{y:.3f}",
            f"{z:.3f}",
            robot,
            mode,
        ]
        with open(self.filepath, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow(row)
        return str(self.filepath)
