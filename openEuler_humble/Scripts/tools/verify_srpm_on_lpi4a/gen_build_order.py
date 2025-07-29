#!/usr/bin/env python3
"""
根据当前目录（或指定目录）里的 *.src.rpm
生成手工 rpmbuild --rebuild 的依赖顺序。

用法：
    python3 gen_build_order.py             # 扫描当前目录
    python3 gen_build_order.py /path/to/   # 扫描指定目录
"""
import os
import re
import sys
import subprocess
from collections import defaultdict, deque

def rpm_query(fmt: str, srpm: str) -> str:
    """调用 rpm -qp --queryformat 获取字段"""
    return subprocess.check_output(
        ["rpm", "-qp", "--queryformat", fmt, srpm],
        text=True
    ).strip()

def get_name(srpm: str) -> str:
    return rpm_query("%{NAME}", srpm)

def get_buildreqs(srpm: str) -> list[str]:
    """提取 BuildRequires（rpm 会把它们列到 requires 里）"""
    out = subprocess.check_output(
        ["rpm", "-qp", "--requires", srpm],
        text=True
    )
    reqs = []
    for line in out.splitlines():
        # 去掉版本比较符号，留下纯包名
        name = re.split(r"[<>=\s]", line, maxsplit=1)[0]
        if name:                                   # 过滤空行
            reqs.append(name)
    return reqs

def topo_sort(pkgs: dict[str, str], deps: dict[str, set[str]]) -> list[str]:
    """Kahn 拓扑排序；返回按依赖递增的包名列表"""
    incoming = {p: len(deps[p]) for p in pkgs}
    q = deque([p for p, deg in incoming.items() if deg == 0])
    order = []
    while q:
        p = q.popleft()
        order.append(p)
        for other in pkgs:
            if p in deps[other]:
                deps[other].remove(p)
                incoming[other] -= 1
                if incoming[other] == 0:
                    q.append(other)
    # 检测环
    cycle = [p for p, deg in incoming.items() if deg > 0]
    if cycle:
        sys.stderr.write(
            f"⚠️  检测到可能的循环依赖：{', '.join(cycle)}\n"
            "    请手动拆环或先编译核心消息/接口包。\n"
        )
    return order

def main(root: str = "."):
    srpms = [os.path.join(root, f) for f in os.listdir(root) if f.endswith(".src.rpm")]
    if not srpms:
        sys.exit("目录里没找到 *.src.rpm")
    names = {get_name(s): s for s in srpms}
    deps = defaultdict(set)
    for name, srpm in names.items():
        br = get_buildreqs(srpm)
        # 只保留在当前 SRPM 集合里的依赖，系统库忽略
        deps[name] = {r for r in br if r in names}
    build_order = topo_sort(names, deps)

    print("# ===== build order =====")
    for n in build_order:
        print(f"rpmbuild --rebuild {os.path.basename(names[n])}")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
