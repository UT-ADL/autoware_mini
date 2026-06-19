# GitLab CLI (glab) Skill for Claude Code

A comprehensive Claude Code skill that provides expert guidance for using the GitLab CLI (`glab`) to manage GitLab resources directly from the command line.

## Overview

This skill enables Claude Code to effectively assist with GitLab workflows using the official `glab` CLI tool. It provides detailed knowledge about GitLab operations including merge requests, issues, CI/CD pipelines, repository management, and more.

## What This Skill Provides

- **Core workflows**: Common GitLab operations (MRs, issues, CI/CD, repos)
- **Authentication guidance**: Quick setup for GitLab.com and self-hosted instances
- **Best practices**: When and how to use glab effectively
- **Progressive disclosure**: Concise core instructions with detailed references loaded as needed
- **Comprehensive references**: Detailed command docs, troubleshooting, and quick reference guides

## Installation

### Installing the Skill

This skill should be placed in your Claude Code skills directory:

```bash
# For project-specific installation
mkdir -p .claude/skills
git clone https://github.com/henricook/claude-glab-skill .claude/skills/glab

# For personal/global installation
mkdir -p ~/.claude/skills
git clone https://github.com/henricook/claude-glab-skill ~/.claude/skills/glab
```

After installation, your directory structure will be:
```
.claude/skills/glab/
├── SKILL.md                          # Core skill (~200 lines, loaded when skill invoked)
├── references/                       # Detailed docs (loaded only as needed)
│   ├── commands-detailed.md          # Comprehensive command reference
│   ├── quick-reference.md            # Command cheat sheet
│   └── troubleshooting.md            # Detailed error scenarios
├── README.md
├── CONTRIBUTING.md
├── CHANGELOG.md
└── LICENSE
```

### Installing glab CLI

Before using this skill, ensure `glab` is installed on your system:

**macOS:**
```bash
brew install glab
```

**Linux:**
```bash
# Debian/Ubuntu
sudo apt install glab

# Fedora/RHEL
sudo dnf install glab

# Arch Linux
sudo pacman -S glab
```

**Windows:**
```powershell
# Using Chocolatey
choco install glab

# Using Scoop
scoop install glab
```

**From source:**
```bash
go install gitlab.com/gitlab-org/cli/cmd/glab@latest
```

For more installation options, visit: https://gitlab.com/gitlab-org/cli

## Usage

Once installed, Claude Code will automatically detect when you need GitLab CLI assistance and can invoke this skill. You can also explicitly invoke it:

```
@claude using the glab skill, help me create a merge request
```

Or simply ask Claude Code to perform GitLab operations:

```
Can you list my open merge requests?
Create an issue for the bug we just found
Show me the status of the CI pipeline
```

## Skill Architecture

This skill follows the **progressive disclosure design principle** for optimal performance:

### Three-Level Context Loading

1. **SKILL.md frontmatter** (~50 chars) - Loaded first for skill discovery and invocation
2. **SKILL.md body** (~200 lines) - Core workflows and patterns loaded when skill is invoked
3. **references/** folder - Detailed documentation loaded into context only as needed

### SKILL.md (Core Instructions)
The main skill file is concise and focused on:
- When to use glab for different tasks
- Essential authentication setup
- Common workflow patterns (not exhaustive command lists)
- Best practices and quick fixes
- References to detailed documentation

**Why it's concise:** Loads quickly when invoked, providing immediate guidance without overwhelming context.

### references/ (Detailed Documentation)

**commands-detailed.md** - Load when:
- User needs specific flag or option details
- Working with advanced commands (API, variables, schedules)
- Need comprehensive command examples

**troubleshooting.md** - Load when:
- Encountering authentication or connection errors
- Debugging CI/CD pipeline issues
- Need detailed error scenarios and solutions

**quick-reference.md** - Load when:
- User wants a command cheat sheet
- Quick lookup of common flags and patterns

### Tool Restrictions
The skill is configured with `allowed-tools: Bash, Read, Grep, Glob` to ensure Claude Code can:
- Execute glab commands via Bash
- Read configuration and reference files as needed
- Search for relevant files and patterns
- Work within the repository context

## Skill Features

### Workflow-First Approach

Rather than memorizing commands, the skill teaches:
- **Creating merge requests** with reviewers and labels
- **Reviewing code** by checking out MRs locally
- **Managing issues** and linking them to MRs
- **Monitoring CI/CD** pipelines and handling failures

### Comprehensive Command Coverage

30+ glab commands documented across:
- Merge Requests, Issues, CI/CD Pipelines
- Repositories, API access, Labels, Releases
- Snippets, Users, Variables, SSH Keys
- And more (see references/commands-detailed.md)

### Context-Aware Assistance

The skill helps Claude Code:
- Detect when authentication is needed
- Identify repository context issues
- Suggest appropriate flags and options
- Load detailed docs only when necessary

### Self-Hosted GitLab Support

Full support for GitLab.com and self-hosted instances with environment variable configuration and multi-instance authentication.

## Examples

After installation, Claude Code can help with tasks like:

**Creating a merge request:**
```
Create a merge request for my current branch with the title "Fix login bug" and assign it to reviewers alice and bob
```

**Reviewing merge requests:**
```
Show me all merge requests where I'm assigned as a reviewer
```

**Managing CI/CD:**
```
Watch the current pipeline and let me know if it passes
```

**Working with issues:**
```
Create a bug issue titled "API timeout" with high priority label
```

## Configuration

The skill automatically adapts to:
- Current repository context
- Authenticated GitLab instances
- Self-hosted GitLab via GITLAB_HOST environment variable
- Multiple authentication profiles

## Contributing

This skill is designed to be comprehensive and up-to-date. If you find commands or workflows that should be added, please contribute:

1. Test your additions with actual glab usage
2. Follow the imperative/infinitive writing style established in SKILL.md
3. Include practical examples
4. Update this README if adding major new sections

## Requirements

- Claude Code with Skills support
- glab CLI tool installed and in PATH
- Authenticated GitLab account (via `glab auth login`)
- Git repository context for repository-specific operations

## Troubleshooting

If Claude Code doesn't recognize the skill:
1. Verify the skill is in `.claude/skills/glab/` or `~/.claude/skills/glab/`
2. Ensure SKILL.md exists and has valid YAML frontmatter
3. Restart Claude Code if necessary

If glab commands fail:
1. Verify installation: `glab --version`
2. Check authentication: `glab auth status`
3. Ensure you're in a Git repository or use `-R owner/repo` flag

## Resources

- **glab Official Repository**: https://gitlab.com/gitlab-org/cli
- **GitLab CLI Documentation**: https://docs.gitlab.com/editor_extensions/gitlab_cli/
- **Claude Code Skills**: https://docs.claude.com/en/docs/claude-code/skills

## License

This skill is provided as a community resource for Claude Code users working with GitLab.

## Version

Version: 1.0.0
Last Updated: November 2025
Compatible with: glab v1.40.0+
