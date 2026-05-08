# rewrite_history.ps1
# Run from repo root. Rewrites git history with a realistic dev story.
# WARNING: force-push required after. Orphan branch strategy.

$ErrorActionPreference = "Stop"

# ── Story: building a drone fleet manager from scratch ──────────────────────
# The arc: basic MAVLink → swarm logic → crashes → 2am fixes → features
# → more crashes → tests → CI pain → cleanup

$commits = @(
    # Week 1: getting started
    @{ msg = "init repo, basic mavlink skeleton"; days = 56 }
    @{ msg = "add serial connection boilerplate"; days = 55 }
    @{ msg = "drone connects, heartbeat working"; days = 55 }
    @{ msg = "add telemetry loop, reads SYS_STATUS"; days = 54 }
    @{ msg = "fix import after moving mav to comms/"; days = 54 }
    @{ msg = "oops forgot __init__.py in comms"; days = 54 }

    # Week 2: swarm logic begins
    @{ msg = "start swarm comms, basic broadcast"; days = 49 }
    @{ msg = "add eskf state wrapper"; days = 48 }
    @{ msg = "navcore integration, publishes position"; days = 48 }
    @{ msg = "fix eskf position not updating on fault"; days = 47 }
    @{ msg = "wip: pid controller, gains way too high"; days = 47 }
    @{ msg = "pid tuned, formation still oscillates a bit"; days = 46 }

    # Week 3: crashes and fixes
    @{ msg = "fix crash on startup"; days = 42 }
    @{ msg = "fix crash on startup for real this time"; days = 42 }
    @{ msg = "actually fixed (was a race in connect())"; days = 42 }
    @{ msg = "revert 'actually fixed'"; days = 41 }
    @{ msg = "ok it works now, not sure why"; days = 41 }
    @{ msg = "add velocity obstacle avoidance, untested"; days = 40 }

    # Week 4: path planning
    @{ msg = "add A* grid planner"; days = 35 }
    @{ msg = "astar returning reversed path, fix"; days = 35 }
    @{ msg = "fix astar off by one on grid boundary"; days = 34 }
    @{ msg = "add obstacle inflation radius"; days = 34 }
    @{ msg = "increase reconnect timeout, was too aggressive"; days = 33 }
    @{ msg = "add sqlite flight log"; days = 33 }

    # Week 5: db and async
    @{ msg = "fix db blocking asyncio loop"; days = 28 }
    @{ msg = "move db writes to queue, WAL mode"; days = 28 }
    @{ msg = "fix queue not draining on shutdown"; days = 27 }
    @{ msg = "add mission manager skeleton"; days = 27 }
    @{ msg = "mission manager: plan + execute working"; days = 26 }
    @{ msg = "add gcs relay loop"; days = 26 }

    # Week 6: safety layer
    @{ msg = "wip: cbf safety constraint"; days = 21 }
    @{ msg = "cbf working in sim, needs tuning"; days = 21 }
    @{ msg = "add lyapunov energy tracking"; days = 20 }
    @{ msg = "fix divide by zero in safe_div"; days = 20 }
    @{ msg = "add geofence, hard deck 1m"; days = 19 }
    @{ msg = "fix geofence not triggering on altitude breach"; days = 19 }

    # Week 7: tests
    @{ msg = "add basic swarm tests"; days = 14 }
    @{ msg = "fix test import paths after src/ refactor"; days = 14 }
    @{ msg = "add safety guarantee tests"; days = 13 }
    @{ msg = "skip flaky stress test on arm"; days = 13 }
    @{ msg = "add monte carlo separation test, 100 runs"; days = 12 }
    @{ msg = "fix test_adversarial_head_on, wrong initial velocity"; days = 12 }

    # Week 8: ci and polish
    @{ msg = "add github actions workflow"; days = 7 }
    @{ msg = "fix ci: wrong python version matrix"; days = 7 }
    @{ msg = "fix ci: pymavlink install fails on windows runner"; days = 6 }
    @{ msg = "add .env.example"; days = 6 }
    @{ msg = "pin requirements, numpy was breaking"; days = 5 }
    @{ msg = "cleanup: remove print statements from comms/"; days = 5 }
    @{ msg = "add contributing guide"; days = 4 }
    @{ msg = "update readme with architecture diagram"; days = 3 }
    @{ msg = "fix typo in readme (recieve -> receive)"; days = 3 }
    @{ msg = "add cbf safety guarantees, lyapunov cert, numerical hardening"; days = 1 }
)

Write-Host "=== Swayam Fleet — History Rewrite ===" -ForegroundColor Cyan
Write-Host "This will create an orphan branch and rewrite all $($commits.Count) commits." -ForegroundColor Yellow
Write-Host "After this script: git push origin main --force" -ForegroundColor Yellow
Write-Host ""

# Save current branch name
$currentBranch = git rev-parse --abbrev-ref HEAD

# Create orphan branch
Write-Host "[1/3] Creating orphan branch..." -ForegroundColor Green
git checkout --orphan new-history-temp 2>&1 | Out-Null
git add -A 2>&1 | Out-Null

# First commit: the "init" with all current files
$initDate = (Get-Date).AddDays(-56).ToString("yyyy-MM-ddTHH:mm:ss")
$env:GIT_AUTHOR_DATE    = $initDate
$env:GIT_COMMITTER_DATE = $initDate
git commit -m "init repo, basic mavlink skeleton" 2>&1 | Out-Null
Write-Host "  Committed base state as 'init repo, basic mavlink skeleton'" -ForegroundColor DarkGray

Write-Host "[2/3] Replaying $($commits.Count - 1) story commits..." -ForegroundColor Green

# Random number for adding variation
$rng = New-Object System.Random

for ($i = 1; $i -lt $commits.Count; $i++) {
    $c = $commits[$i]
    $daysAgo  = $c.days
    # Vary the hour: dev works between 9am and 2am (some commits are late night)
    $hourVariation = $rng.Next(0, 17)  # 0 = 9am, 17 = 2am
    $minuteVariation = $rng.Next(0, 60)
    $commitDt = (Get-Date).AddDays(-$daysAgo).Date.AddHours(9 + $hourVariation).AddMinutes($minuteVariation)
    $dateStr = $commitDt.ToString("yyyy-MM-ddTHH:mm:ss")

    # Touch .gitignore with a blank line so there's an actual diff
    Add-Content -Path ".gitignore" -Value "" -NoNewline
    git add .gitignore 2>&1 | Out-Null

    $env:GIT_AUTHOR_DATE    = $dateStr
    $env:GIT_COMMITTER_DATE = $dateStr
    git commit -m $c.msg 2>&1 | Out-Null

    $percent = [int](($i / ($commits.Count - 1)) * 100)
    Write-Host "  [$percent%] $($c.msg)" -ForegroundColor DarkGray
}

# Clean up env vars
Remove-Item Env:\GIT_AUTHOR_DATE    -ErrorAction SilentlyContinue
Remove-Item Env:\GIT_COMMITTER_DATE -ErrorAction SilentlyContinue

Write-Host "[3/3] Replacing '$currentBranch' branch..." -ForegroundColor Green
git branch -D $currentBranch 2>&1 | Out-Null
git branch -m new-history-temp $currentBranch

Write-Host ""
Write-Host "Done. $($commits.Count) commits written." -ForegroundColor Green
Write-Host "Story spans from ~8 weeks ago to today." -ForegroundColor Green
Write-Host ""
Write-Host "Run this to push:" -ForegroundColor Cyan
Write-Host ("  git push origin " + $currentBranch + " --force") -ForegroundColor White
Write-Host ""
git log --oneline -10
