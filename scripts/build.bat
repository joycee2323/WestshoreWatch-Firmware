@echo off
set MSYSTEM=
set MSYS=
set MINGW_PREFIX=
set IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.5.3
set IDF_TOOLS_PATH=C:\Espressif
set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf5.5_py3.11_env
set ESP_ROM_ELF_DIR=C:\Espressif\tools\esp-rom-elfs\20241011\
set "PATH=C:\Espressif\python_env\idf5.5_py3.11_env\Scripts;C:\Espressif\tools\idf-git\2.44.0\cmd;C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20251107\riscv32-esp-elf\bin;C:\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64;%PATH%"
cd /d C:\Users\joyce\Westshore Watch\C5-Firmware
python "%IDF_PATH%\tools\idf.py" %*
exit /b %errorlevel%
