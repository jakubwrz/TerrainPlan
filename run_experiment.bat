@echo off
echo ==================================================
echo [1/3] Starting TerrainPlan Analysis...
echo ==================================================
python TerrainPlan\TerrainPlan.py
if %errorlevel% neq 0 (
    echo.
    echo ERROR: TerrainPlan failed. Please check the map image and markers.
    pause
    exit /b %errorlevel%
)

echo.
echo ==================================================
echo [2/3] Running A* Path Planner (Fetching Live Rover Start)...
echo ==================================================
python astar.py --use-rover-start
if %errorlevel% neq 0 (
    echo.
    echo ERROR: A* Path Planning failed. Ensure the rover is powered on and streaming MQTT.
    pause
    exit /b %errorlevel%
)

echo.
echo ==================================================
echo [3/3] Starting UWB Server and Motor Control...
echo ==================================================
python TerrainPlan\uwb_server.py

pause
