"""llm.py 的离线单元测试：不联网、不调大模型，只测文本修正与结果校验。"""

import unittest

from . import llm


class RepairZeroTests(unittest.TestCase):
    def test_zero_misheard_as_lin_is_repaired(self):
        self.assertEqual(llm._repair_zero("去林号格"), "去0号格")
        self.assertEqual(llm._repair_zero("去林浩格"), "去0号格")
        self.assertEqual(llm._repair_zero("到林好位"), "到0号位")

    def test_other_numbers_are_untouched(self):
        self.assertEqual(llm._repair_zero("去三号格"), "去三号格")
        self.assertEqual(llm._repair_zero("去六号格"), "去六号格")


class ParseTests(unittest.TestCase):
    def test_allowed_cells(self):
        for cell in "03678":
            with self.subTest(cell=cell):
                raw = '{"action":"arrive","number":"%s","confidence":0.9}' % cell
                self.assertEqual(llm._parse(raw), (cell, "arrive"))

    def test_unreachable_cells_are_rejected(self):
        for cell in "1245":
            with self.subTest(cell=cell):
                raw = '{"action":"arrive","number":"%s","confidence":0.9}' % cell
                with self.assertRaises(llm.LLMParseError):
                    llm._parse(raw)

    def test_zero_homophone_returned_by_model(self):
        for number in ("林", "林浩", "林号", "零", "O"):
            with self.subTest(number=number):
                raw = '{"action":"arrive","number":"%s","confidence":0.9}' % number
                self.assertEqual(llm._parse(raw), ("0", "arrive"))

    def test_empty_command(self):
        raw = '{"action":null,"number":null,"confidence":0.9}'
        self.assertEqual(llm._parse(raw), (None, None))

    def test_salvage_from_truncated_json(self):
        truncated = '{"action":"arrive","number":"7","confidence":0.'
        self.assertEqual(llm._parse(truncated), ("7", "arrive"))
        truncated_4 = '{"action":"arrive","number":"4","confidence":0.'
        self.assertRaises(llm.LLMParseError, llm._parse, truncated_4)


class PromptTests(unittest.TestCase):
    def test_prompt_lists_only_reachable_cells(self):
        self.assertIn("0、3、6、7、8", llm.SYSTEM_PROMPT)


if __name__ == "__main__":
    unittest.main()
