# Changelog

All notable changes to the glab skill for Claude Code will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-11-05

### Added
- Initial release of the glab skill for Claude Code
- Comprehensive SKILL.md with detailed command documentation
- Coverage of core glab commands:
  - Authentication (`glab auth`)
  - Merge Requests (`glab mr`)
  - Issues (`glab issue`)
  - CI/CD (`glab ci`, `glab pipeline`)
  - Repository operations (`glab repo`)
  - API access (`glab api`)
  - Labels, Releases, Snippets, Users
- Common workflow patterns:
  - Feature branch and MR workflow
  - Code review process
  - CI/CD pipeline monitoring
  - Issue tracking and linking
- Best practices and troubleshooting guidance
- Quick reference guide in `references/quick-reference.md`
- Comprehensive README.md with installation and usage instructions
- CONTRIBUTING.md with contribution guidelines
- MIT License
- .gitignore for common development artifacts

### Features
- Tool restrictions (`allowed-tools: Bash, Read, Grep, Glob`)
- Support for both GitLab.com and self-hosted instances
- Environment variable configuration (GITLAB_TOKEN, GITLAB_HOST)
- Multi-instance authentication support
- 30+ documented glab commands
- Real-world usage examples throughout

### Documentation
- Complete command reference with examples
- Common flags and options explained
- Project context detection guidance
- Configuration management instructions
- Error messages and solutions

## [1.1.0] - 2025-11-06

### Changed - Progressive Disclosure Refactoring

**Major restructuring following Claude Code Skills best practices:**

- **SKILL.md reduced from 471 to 203 lines** - Now focused on core workflows and patterns
- **Implemented three-level context loading:**
  1. SKILL.md frontmatter for discovery
  2. SKILL.md body for core workflows (loaded when invoked)
  3. references/ folder for detailed docs (loaded only as needed)

### Added
- **references/commands-detailed.md** (610 lines) - Comprehensive command reference with all flags and options
- **references/troubleshooting.md** (669 lines) - Detailed error scenarios, causes, and solutions

### Improved
- **SKILL.md** now workflow-first, teaching WHEN and HOW to use glab
- **README** now explains progressive disclosure architecture
- **references/quick-reference.md** remains as condensed cheat sheet (145 lines)

### Why This Matters
- **Faster skill invocation** - Smaller initial context load
- **Better performance** - Claude loads detailed docs only when needed
- **Clearer guidance** - Workflows over command memorization
- **Scalable architecture** - Easy to add more detailed references

### Migration Notes
Users should re-pull or re-clone the skill to get the new structure:
```bash
cd ~/.claude/skills/glab && git pull
# or
rm -rf ~/.claude/skills/glab && git clone <repo> ~/.claude/skills/glab
```

## [Unreleased]

### Planned
- Additional examples for advanced workflows
- More coverage of `glab duo` AI features
- Enterprise GitLab feature documentation
- Integration patterns with CI/CD tools
- Scripts for common operations

## Notes

- This skill is based on glab v1.40.0+ command structure
- Tested with Claude Code Skills specification
- Designed following progressive disclosure principles
- Written in imperative/infinitive style as per Claude Code best practices

---

For the complete list of changes, see the [commit history](../../commits/main).
