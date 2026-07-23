# Stop immediately if any command fails.
$ErrorActionPreference = "Stop"

# -----------------------------------------------------------------------------
# Source directories
# -----------------------------------------------------------------------------

$GeneratedCodeDirectory = "\\wsl.localhost\Ubuntu\home\tinta\Amon_lander\Model_CasADi\amon\model_casadi_acados\acados_generated\c_generated_code"
$ModelDirectory = Join-Path $GeneratedCodeDirectory "amon_model_model"

# -----------------------------------------------------------------------------
# Destination directory
# -----------------------------------------------------------------------------

$DestinationDirectory = "C:\DATA\Projects\Amon_Board\Firmware\Amon_board_firmware-STM32H743\acados"

# -----------------------------------------------------------------------------
# Files located directly inside c_generated_code
# -----------------------------------------------------------------------------

$GeneratedCodeFiles = @(
    "acados_solver_amon_model.c",
    "acados_solver_amon_model.h"
)

# -----------------------------------------------------------------------------
# Files located inside c_generated_code\amon_model_model
# -----------------------------------------------------------------------------

$ModelFiles = @(
    "amon_model_expl_ode_fun.c",
    "amon_model_expl_vde_adj.c",
    "amon_model_expl_vde_forw.c",
    "amon_model_model.h"
)

function Copy-CheckedFile {
    param(
        [Parameter(Mandatory)]
        [string] $SourceFile,

        [Parameter(Mandatory)]
        [string] $DestinationDirectory
    )

    if (-not (Test-Path -LiteralPath $SourceFile -PathType Leaf)) {
        throw "Source file does not exist: $SourceFile"
    }

    Write-Host "Copying: $SourceFile"

    Copy-Item `
        -LiteralPath $SourceFile `
        -Destination $DestinationDirectory `
        -Force
}

function Replace-ExactText {
    param(
        [Parameter(Mandatory)]
        [string] $Content,

        [Parameter(Mandatory)]
        [string] $OldText,

        [Parameter(Mandatory)]
        [string] $NewText,

        [Parameter(Mandatory)]
        [string] $Description
    )

    $OccurrenceCount = [regex]::Matches(
        $Content,
        [regex]::Escape($OldText)
    ).Count

    if ($OccurrenceCount -eq 0) {
        throw "Could not find the expected text for: $Description"
    }

    if ($OccurrenceCount -gt 1) {
        throw "Found the expected text $OccurrenceCount times for '$Description'; exactly one occurrence was expected."
    }

    return $Content.Replace($OldText, $NewText)
}

try {
    Write-Host ""
    Write-Host "Updating generated acados files..."
    Write-Host "Source:      $GeneratedCodeDirectory"
    Write-Host "Destination: $DestinationDirectory"
    Write-Host ""

    # Check that the main source directory is accessible.
    if (-not (Test-Path -LiteralPath $GeneratedCodeDirectory -PathType Container)) {
        throw "Generated-code directory is not accessible: $GeneratedCodeDirectory"
    }

    # Check that the model source directory is accessible.
    if (-not (Test-Path -LiteralPath $ModelDirectory -PathType Container)) {
        throw "Model directory is not accessible: $ModelDirectory"
    }

    # Create the destination directory if it does not exist.
    if (-not (Test-Path -LiteralPath $DestinationDirectory -PathType Container)) {
        Write-Host "Creating destination directory..."

        New-Item `
            -ItemType Directory `
            -Path $DestinationDirectory `
            -Force | Out-Null
    }

    # Copy files from c_generated_code.
    foreach ($FileName in $GeneratedCodeFiles) {
        $SourceFile = Join-Path $GeneratedCodeDirectory $FileName

        Copy-CheckedFile `
            -SourceFile $SourceFile `
            -DestinationDirectory $DestinationDirectory
    }

    # Copy files from c_generated_code\amon_model_model.
    foreach ($FileName in $ModelFiles) {
        $SourceFile = Join-Path $ModelDirectory $FileName

        Copy-CheckedFile `
            -SourceFile $SourceFile `
            -DestinationDirectory $DestinationDirectory
    }

    Write-Host ""
    Write-Host "All acados files were copied successfully."
    Write-Host ""

    # -------------------------------------------------------------------------
    # Modify acados_solver_amon_model.c
    # -------------------------------------------------------------------------

    $SolverFile = Join-Path $DestinationDirectory "acados_solver_amon_model.c"

    if (-not (Test-Path -LiteralPath $SolverFile -PathType Leaf)) {
        throw "Copied solver file does not exist: $SolverFile"
    }

    Write-Host "Modifying: $SolverFile"

    $Content = Get-Content -LiteralPath $SolverFile -Raw

    # Change:
    # #include "amon_model_model/amon_model_model.h"
    # to:
    # #include "amon_model_model.h"
    $Content = Replace-ExactText `
        -Content $Content `
        -OldText '#include "amon_model_model/amon_model_model.h"' `
        -NewText '#include "amon_model_model.h"' `
        -Description "amon_model_model.h include path"

    # Disable allocation of capsule->sens_out and explicitly set it to NULL.
    $OldAllocation = '    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);'

    $NewAllocation = @'
    // capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    capsule->sens_out = NULL;
'@

    $Content = Replace-ExactText `
        -Content $Content `
        -OldText $OldAllocation `
        -NewText $NewAllocation `
        -Description "sens_out allocation"

    # Comment out the unconditional destruction and replace it with a NULL check.
    $OldDestruction = '    ocp_nlp_out_destroy(capsule->sens_out);'

    $NewDestruction = @'
    // ocp_nlp_out_destroy(capsule->sens_out);
    if (capsule->sens_out != NULL)
    {
        ocp_nlp_out_destroy(capsule->sens_out);
    }
'@

    $Content = Replace-ExactText `
        -Content $Content `
        -OldText $OldDestruction `
        -NewText $NewDestruction `
        -Description "sens_out destruction"

    # ASCII is suitable for the generated C source and avoids UTF-16 output in
    # Windows PowerShell 5.1.
    Set-Content `
        -LiteralPath $SolverFile `
        -Value $Content `
        -Encoding ASCII `
        -NoNewline

    Write-Host "Finished modifying acados_solver_amon_model.c"
    Write-Host ""
    Write-Host "Acados file update completed successfully."
    Write-Host ""
}
catch {
    Write-Host ""
    Write-Error "Acados file update failed: $($_.Exception.Message)"
    exit 1
}
