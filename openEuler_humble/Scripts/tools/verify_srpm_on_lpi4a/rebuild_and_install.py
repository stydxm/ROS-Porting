#!/usr/bin/env python3
"""
rebuild_and_install.py  (增量安装 + pending 队列)
"""

import argparse, re, shlex, subprocess, sys, time
from pathlib import Path
from typing import List

# --- 根据实际路径自定义 ---------------------------------------------------
PKG_DIR  = Path("/home/openeuler/ros-srpm/openEuler-ROS-src/Packages")
BUILT    = PKG_DIR / "built"
RPM_DIR  = Path("/home/openeuler/rpmbuild/RPMS")
# -------------------------------------------------------------------------

def run(cmd, sudo=False, cap=False):
    if sudo: cmd = ["sudo"] + cmd
    return subprocess.run(cmd, text=True, capture_output=cap)

# ---------- rpmbuild ------------------------------------------------------

def build_one(cmd: List[str], sudo_dnf: bool) -> bool:
    srpm = cmd[-1]
    run(["dnf","builddep","-y",srpm], sudo_dnf)
    for try_ in (1,2):
        start = time.time()
        res = run(cmd, cap=True)
        if res.returncode==0: return True
        if try_==1:
            miss=re.findall(r"^\s*([\w.+-]+)\s+is needed by",res.stderr,re.M)
            if miss:
                run(["dnf","install","-y",*miss], sudo_dnf); continue
        print(res.stderr); return False
    return False

# ---------- 主逻辑 --------------------------------------------------------

def main(order_file:str, sudo_dnf:bool):
    pending: List[str] = []        # 待安装 rpm
    failures: List[str] = []

    def try_install(rpms:List[str]):
        if not rpms: return []
        res = run(["dnf","install","-y",*rpms], sudo_dnf, cap=True)
        if res.returncode==0:
            print(f"🟢 installed {len(rpms)} rpm")
            return []
        # dnf 报缺依赖 → 把无法安装的 rpm 留到下一轮
        print("🔄  postpone install, unmet deps")
        return rpms

    for line in Path(order_file).read_text().splitlines():
        if not line.startswith("rpmbuild --rebuild"): continue
        cmd = shlex.split(line.strip())
        srpm = PKG_DIR / Path(cmd[-1]).name
        done = BUILT / srpm.name
        if done.exists(): continue

        # 若之前已生成 rpm 但依赖未满足 → 仍放 pending
        for rpm_file in RPM_DIR.rglob("*.rpm"):
            if rpm_file.stat().st_mtime >= srpm.stat().st_mtime-5:
                pending.append(str(rpm_file))

        print(f"\n🚧 build {srpm.name}")
        ok = build_one([*cmd[:-1], str(srpm)], sudo_dnf)
        if not ok:
            failures.append(srpm.name); continue

        # 收集本轮新 rpm
        new_rpms=[str(p) for p in RPM_DIR.rglob("*.rpm")
                  if p.stat().st_mtime >= time.time()-10]
        pending.extend(new_rpms)

        # 立即尝试安装队列
        pending = try_install(list(dict.fromkeys(pending)))  # 去重

        # 归档 srpm
        BUILT.mkdir(exist_ok=True); srpm.rename(done)

    # 最后一轮安装
    pending = try_install(pending)
    if pending:
        print("⚠️  一些 rpm 仍装不上，请手动检查:", *pending, sep="\n  ")

    if failures:
        print("\n❌ build failed:", *failures, sep="\n  ")
        sys.exit(1)
    print("\n🎉 done")

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("order_file")
    ap.add_argument("--sudo",action="store_true")
    a=ap.parse_args()
    main(a.order_file,a.sudo)
