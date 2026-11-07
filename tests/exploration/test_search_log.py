from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/edgeai_navigation/scripts'))
from search_log import search_text


class SearchLog(unittest.TestCase):
    def test_source_label_and_rounded_coordinates_without_changing_data(self):
        candidate=dict(strategy='suspected_panel',suspect_id=2,x=3.5535050508712147,
            y=-1.4900341789128286,yaw=1.5668793356631951,
            suspect_position=[3.556769201800738,-.6567072384127908,.2534203224259647])
        text=search_text('SEARCH_CANDIDATE',candidate)
        self.assertIn('疑似目标候选 #2',text)
        self.assertIn('map观察位=(3.55, -1.49) m',text)
        self.assertIn('疑似位置=(3.56, -0.66, 0.25) m',text)
        self.assertNotIn('553505',text)
        self.assertEqual(candidate['x'],3.5535050508712147)
        candidate.update(strategy='frustum',gain_m2=2.2533334004879)
        self.assertIn('视锥候选',search_text('SEARCH_CANDIDATE',candidate))
        self.assertIn('2.25 m²',search_text('SEARCH_CANDIDATE',candidate))

    def test_spin_keeps_source_and_terminal_reason(self):
        text=search_text('SEARCH_SPIN_RESULT',dict(strategy='frustum',x=2.240206395054305,
            y=-3.8071282468066396,status=5,error_code=0,cancel_reason='target_found'))
        self.assertIn('视锥候选',text)
        self.assertIn('转圈已取消',text)
        self.assertIn('已发现目标',text)


if __name__=='__main__':unittest.main()
