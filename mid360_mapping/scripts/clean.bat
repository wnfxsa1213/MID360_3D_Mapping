@echo off
REM MID360 3D建图系统清理脚本 (Windows版)

REM 清理构建目录
if exist build (
    echo 正在清理build目录...
    rmdir /S /Q build
    echo 清理完成！
) else (
    echo build目录不存在，无需清理
)

echo 清理过程完成！ 