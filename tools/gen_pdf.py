from fpdf import FPDF
import json
import cv2


class PdfGenerator:
    def __init__(self, json_file="report.json") -> None:
        with open(json_file) as f:
            self.json_reader = json.load(f)
        self.title = self.json_reader['title']
        self.project_name = self.json_reader['project']
        self.pass_count_img = cv2.imread(self.json_reader['pass_count_img'])
        # TO-DO: Area
        # TO-DO: Temp



