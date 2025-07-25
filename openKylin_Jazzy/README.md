# README

## File Structure

```
➜  openKylin_Jazzy git:(main) ✗ tree
.
├── README.md
├── ros_core -> variant
│   └── buildrequire_list_raw.json
└── scripts -> Deps verify scripts for openKylin
    ├── extract_buildrequires.py
    └── verify_packages.py

3 directories, 4 files
```

## verify_packages.py

usage
```
mkdir verify_json

mv verify_packages.py verify_json

mv buildrequire_list_raw.json verify_json

# 自动搜索 JSON 文件 -> default
python3 verify_packages.py

# 指定 JSON 文件路径
python3 verify_packages.py -i ~/verify_json/buildrequire_list_raw.json

# 启用详细输出
python3 verify_packages.py -v

# 跳过 apt update
python3 verify_packages.py --skip-update
```
