param(
    [string]$OpenScad = "C:\Program Files\OpenSCAD\openscad.com"
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot

function Export-OpenScadPart {
    param(
        [string]$Source,
        [string]$Variable,
        [string]$Selector,
        [string]$Output
    )
    $outputDir = Split-Path -Parent $Output
    New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
    Write-Host "Rendering $Output ($Variable=$Selector)"
    & $OpenScad -o $Output -D "$Variable=`"$Selector`"" $Source
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $Output)) {
        throw "OpenSCAD export failed: $Output"
    }
}

$hamsterSource = Join-Path $repo "v0.9\hamster_print_ready_v0.9.scad"
$hamsterOut = Join-Path $repo "v0.9\stl_print"
$hamsterParts = @(
    @("chassis", "01_upper_shell.stl"),
    @("lower_chassis_with_upper_mounts", "02_lower_frame_STABILITY_SKIDS.stl"),
    @("lid", "03_lid.stl"),
    @("gate_slider", "04_gate_slider_08MM_FLOOR.stl"),
    @("guide_cap", "05_guide_cap_x2.stl"),
    @("servo_mount", "06_servo_mount.stl"),
    @("servo_clamp", "07_servo_clamp.stl"),
    @("connecting_link", "08_connecting_link.stl"),
    @("left_motor_mount", "09_motor_mount_left_N20.stl"),
    @("right_motor_mount", "10_motor_mount_right_N20.stl"),
    @("electronics_deck", "11_electronics_deck.stl"),
    @("battery_mount", "12_battery_tray_SCX24_350mAh.stl"),
    @("caster_mount", "13_caster_mount.stl")
)
foreach ($part in $hamsterParts) {
    Export-OpenScadPart $hamsterSource "part_to_render" $part[0] (Join-Path $hamsterOut $part[1])
}
Export-OpenScadPart $hamsterSource "part_to_render" "assembly" `
    (Join-Path $repo "v0.9\hamster_v0.9_printed_structure_REVIEW_ONLY.stl")

$hamsterTestOut = Join-Path $repo "v0.9\stl_test_first"
Export-OpenScadPart $hamsterSource "part_to_render" "fit_coupon" `
    (Join-Path $hamsterTestOut "fit_coupon.stl")
Export-OpenScadPart $hamsterSource "part_to_render" "guide_test_key" `
    (Join-Path $hamsterTestOut "guide_test_key.stl")

$beaverSource = Join-Path $repo "beaver_v0.7_hardware\beaver_robot_v0.7_hardware.scad"
$beaverOut = Join-Path $repo "beaver_v0.7_hardware\stl_print"
$beaverParts = @(
    @("integrated_chassis", "01_integrated_chassis_N20_DRIVE.stl"),
    @("jaw_left", "02_jaw_left_M25_CLEARANCE.stl"),
    @("jaw_right", "03_jaw_right_M25_CLEARANCE.stl"),
    @("slider_yoke", "04_slider_yoke_M25_CLEARANCE.stl"),
    @("link", "05_link_x2_M25_CLEARANCE.stl"),
    @("servo_mount", "06_front_servo_mount.stl"),
    @("cargo_magazine", "07_cargo_magazine_STEEPER.stl"),
    @("rear_dump_gate", "08_rear_dump_gate_STRONG_KICK_CAM.stl"),
    @("dump_servo_mount", "09_dump_servo_mount.stl"),
    @("upper_housing", "10_removable_upper_housing_STRONG_BOSSES.stl"),
    @("battery_tray", "11_battery_tray_SCX24_350mAh.stl"),
    @("motor_mount_left", "12_motor_mount_left_N20.stl"),
    @("motor_mount_right", "13_motor_mount_right_N20.stl")
)
foreach ($part in $beaverParts) {
    Export-OpenScadPart $beaverSource "part" $part[0] (Join-Path $beaverOut $part[1])
}
Export-OpenScadPart $beaverSource "part" "printed_structure_review" `
    (Join-Path $repo "beaver_v0.7_hardware\beaver_v0.7_printed_structure_REVIEW_ONLY.stl")

$beaverRefOut = Join-Path $repo "beaver_v0.7_hardware\reference"
Export-OpenScadPart $beaverSource "part" "patient_reference" `
    (Join-Path $beaverRefOut "patient_cylinder_30x20_REFERENCE_ONLY.stl")
Export-OpenScadPart $beaverSource "part" "box_reference" `
    (Join-Path $beaverRefOut "cargo_box_25x25x20_REFERENCE_ONLY.stl")
Export-OpenScadPart $beaverSource "part" "wheel_reference" `
    (Join-Path $beaverRefOut "wheel_43x17_5_REFERENCE_ONLY.stl")

Write-Host "Hardware CAD exports complete."
