from _pytest.terminal import TerminalReporter


def pytest_terminal_summary(terminalreporter: TerminalReporter, exitstatus, config):
    """在测试结束时输出周期数统计"""
    cycle_stats = getattr(config, "cycle_stats", None)
    if cycle_stats and cycle_stats.stats:
        terminalreporter.section("MIPS 执行周期统计")
        for test_name, cycles in sorted(cycle_stats.stats.items()):
            terminalreporter.write_line(f"{test_name}: {cycles} 周期")
