@echo off
echo 开始测试CMake配置...

REM 创建测试目录
if not exist test_build mkdir test_build
cd test_build

REM 尝试最小配置
echo 正在使用最小配置进行测试...
cmake -S .. -B . -G "Ninja" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER="C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.43.34808/bin/HostX64/x64/cl.exe" -DCMAKE_C_COMPILER="C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.43.34808/bin/HostX64/x64/cl.exe" -DCMAKE_PREFIX_PATH="D:/QT6/6.9.0/msvc2022_64" > cmake_output.log 2>&1

REM 检查结果
if %ERRORLEVEL% == 0 (
    echo CMake配置成功！
) else (
    echo CMake配置失败，错误码: %ERRORLEVEL%
    echo 请查看 test_build/cmake_output.log 文件获取详细错误信息
)

cd .. 