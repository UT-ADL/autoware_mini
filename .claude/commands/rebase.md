---
allowed-tools: Bash(git:*), Read, Edit, Glob, Grep
description: Rebase current branch onto main and fix merge conflicts
args: source_branch
---

# Rebase onto Source Branch

Rebase the current branch onto the latest $ARGUMENTS (default: main) branch and resolve any merge conflicts.

## Process

1. First, fetch the latest changes from origin
2. Check for uncommitted changes and stash if needed
3. Rebase onto origin/$ARGUMENTS (use origin/main if no argument provided)
4. If conflicts occur:
   - Identify conflicting files
   - Read and understand each conflict
   - Fix conflicts by editing files (choose the correct resolution)
   - Stage resolved files
   - Continue the rebase
5. Repeat until rebase is complete

## Conflict Resolution Strategy

When resolving conflicts:
- Read both versions (HEAD = current branch, incoming = source branch)
- Understand what each change is trying to accomplish
- Merge the changes intelligently, keeping functionality from both sides when appropriate
- Remove all conflict markers (<<<<<<<, =======, >>>>>>>)
- Test that the resolved code makes sense

## Execute

Fetch latest and rebase onto the source branch. Fix any conflicts that arise by editing the files directly, then continue the rebase until complete.
