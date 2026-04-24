$env:IDF_PATH='C:\espressif\frameworks\esp-idf-v5.5.3'
$env:IDF_TOOLS_PATH='C:\espressif'
$env:IDF_PYTHON_ENV_PATH='C:\espressif\python_env\idf5.5_py3.11_env'
$script:IdfPython='C:\espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe'
$exports = & $script:IdfPython "$env:IDF_PATH\tools\idf_tools.py" export --format key-value 2>$null
foreach ($line in $exports) {
    if ($line -match '^([^=]+)=(.*)$') {
        $k = $Matches[1]
        $v = $Matches[2] -replace '%PATH%', $env:PATH
        Set-Item -Path "env:$k" -Value $v
    }
}
function idfpy { & $script:IdfPython "$env:IDF_PATH\tools\idf.py" @args }
