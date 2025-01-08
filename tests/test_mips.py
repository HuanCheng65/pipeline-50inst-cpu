import re
import subprocess
from pathlib import Path

import pytest
from _pytest.terminal import TerminalReporter


def get_all_asm_files():
    """获取所有的.asm测试文件"""
    asm_dir = Path("../mips-asm-test")
    return list(asm_dir.glob("*.asm"))


def get_cycle_count():
    """从pipeline_debug.log末尾高效地获取总执行周期数"""
    try:
        with open("pipeline_debug.log", "rb") as f:
            # 从文件末尾开始读取最后1KB的数据
            CHUNK_SIZE = 1024
            f.seek(0, 2)  # 移动到文件末尾
            size = f.tell()

            chunk_size = min(CHUNK_SIZE, size)
            f.seek(size - chunk_size)
            chunk = f.read(chunk_size).decode("utf-8", errors="ignore")

            # 查找最后一个周期数
            if match := re.search(
                r"========== Cycle (\d+) ==========(?!.*========== Cycle)", chunk
            ):
                return int(match.group(1))
    except FileNotFoundError:
        return None
    return None


# 存储所有测试的周期数
class CycleStats:
    def __init__(self):
        self.stats = {}

    def add_result(self, test_name, cycles):
        self.stats[test_name] = cycles


@pytest.fixture(scope="session")
def cycle_stats(request):
    stats = CycleStats()
    request.config.cycle_stats = stats  # 将统计对象存储在config中
    return stats


def pytest_terminal_summary(terminalreporter: TerminalReporter, exitstatus, config):
    """在测试结束时输出周期数统计"""
    cycle_stats = getattr(config, "cycle_stats", None)
    if cycle_stats and cycle_stats.stats:
        terminalreporter.section("MIPS 执行周期统计")
        for test_name, cycles in cycle_stats.stats.items():
            terminalreporter.write_line(f"{test_name}: {cycles} 周期")


@pytest.mark.parametrize("asm_file", get_all_asm_files(), ids=lambda x: x.stem)
def test_mips_asm(asm_file, cycle_stats, request):
    """测试单个.asm文件"""
    cmd = [
        "python",
        "../pipeline-tester-py/main.py",
        "run",
        "./mips_tb.v",
        str(asm_file),
    ]

    result = subprocess.run(cmd, capture_output=True, text=True)
    assert "PASSED" in result.stdout, f"测试失败: {asm_file.name}\n{result.stdout}"

    # 获取并记录周期数
    cycles = get_cycle_count()
    if cycles is not None:
        cycle_stats.add_result(asm_file.stem, cycles)
