param(
    [string]$Webots = 'C:/Program Files/Webots/msys64/mingw64/bin/webots.exe'
)

$ErrorActionPreference = 'Stop'
$drone = Split-Path $PSScriptRoot -Parent
$project = Join-Path ([System.IO.Path]::GetTempPath()) "uav-avoidance-$([guid]::NewGuid())"
$controller = Join-Path $project 'controllers/flight_avoidance'
New-Item -ItemType Directory -Path "$project/worlds", $controller | Out-Null
foreach ($name in 'drone', 'hybrid', 'wheels') {
    Copy-Item "$drone/controllers/$name" "$project/controllers" -Recurse
}
Copy-Item "$drone/textures" "$project/textures" -Recurse
Copy-Item "$PSScriptRoot/flight_avoidance.py" $controller
$world = Join-Path $project 'worlds/avoidance.wbt'
$worldText = [System.IO.File]::ReadAllText("$drone/worlds/hybrid_wheels_ros2.wbt").Replace(
    'controller "wheels"', 'controller "flight_avoidance"'
)
[System.IO.File]::WriteAllText($world, $worldText)

Write-Output "Evidence directory $project"
$oldSource = $env:UAV_AVOIDANCE_SOURCE
$oldOutput = $env:UAV_AVOIDANCE_OUTPUT
try {
    foreach ($source in 'camera', 'lidar') {
        $env:UAV_AVOIDANCE_SOURCE = $source
        $env:UAV_AVOIDANCE_OUTPUT = Join-Path $project "$source.json"
        $run = Start-Process -FilePath $Webots -ArgumentList @(
            '--batch', '--mode=fast', '--no-rendering', ('"{0}"' -f $world)
        ) -WindowStyle Hidden -PassThru
        if (-not $run.WaitForExit(90000)) {
            taskkill /PID $run.Id /T /F
            throw "Webots timed out for $source"
        }
        $result = Get-Content -LiteralPath $env:UAV_AVOIDANCE_OUTPUT -Raw
        Write-Output $result
        if (-not ($result | ConvertFrom-Json).passed) {
            throw "Webots avoidance check failed for $source"
        }
    }
}
finally {
    $env:UAV_AVOIDANCE_SOURCE = $oldSource
    $env:UAV_AVOIDANCE_OUTPUT = $oldOutput
}
