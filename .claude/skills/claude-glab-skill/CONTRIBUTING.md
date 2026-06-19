# Contributing to the glab Skill

Thank you for your interest in improving the GitLab CLI skill for Claude Code! This document provides guidelines for contributing.

## How to Contribute

### Reporting Issues

If you find errors, missing commands, or unclear instructions:

1. Check if the issue already exists in the issue tracker
2. Provide specific details about the problem
3. Include the glab version you're using (`glab --version`)
4. Include examples of commands that don't work as documented

### Suggesting Improvements

We welcome suggestions for:
- Additional glab commands to document
- New workflow patterns
- Better examples
- Clearer explanations
- Additional troubleshooting scenarios

### Making Changes

1. **Test First**: Always test commands with actual glab CLI before documenting
2. **Follow the Style**: Match the existing writing style (imperative/infinitive form)
3. **Include Examples**: Provide practical, working examples
4. **Update References**: If adding major sections, update the quick-reference guide

## Style Guidelines

### Writing Style

Follow the established imperative/infinitive form:
- ✅ "To create a merge request, use: `glab mr create`"
- ❌ "You can create a merge request by using `glab mr create`"

### Code Examples

- Always use proper markdown code fencing with bash syntax highlighting
- Include comments for complex commands
- Show expected output when relevant
- Test all examples before committing

### Organization

The SKILL.md is organized as:
1. **YAML frontmatter** - Metadata (name, description, allowed-tools)
2. **Introduction** - What the skill provides
3. **When to Use** - Invocation scenarios
4. **Prerequisites** - Setup requirements
5. **Core Commands** - Organized by feature area
6. **Workflows** - Complete usage patterns
7. **Best Practices** - Guidance and recommendations
8. **Troubleshooting** - Common issues and solutions

When adding content, place it in the appropriate section.

## Testing Changes

Before submitting changes:

1. **Verify YAML frontmatter** is valid:
   - `name` is lowercase with hyphens only
   - `description` is clear and under 1024 characters
   - `allowed-tools` list is appropriate

2. **Test commands** with actual glab CLI:
   ```bash
   # Verify glab is installed
   glab --version

   # Test commands you've documented
   glab <command> --help
   ```

3. **Check markdown formatting**:
   - Code blocks are properly fenced
   - Lists are consistently formatted
   - Links work correctly

4. **Verify skill loads in Claude Code**:
   - Place skill in `.claude/skills/glab/`
   - Restart Claude Code
   - Test skill invocation

## What to Update

When making changes, consider updating:

- **SKILL.md** - Main skill instructions
- **quick-reference.md** - If adding major commands
- **README.md** - If changing installation or usage
- **CHANGELOG.md** - Document your changes

## Commit Guidelines

Write clear commit messages:
- ✅ "Add glab duo commands documentation"
- ✅ "Fix incorrect flag for glab mr create"
- ✅ "Update authentication examples"
- ❌ "Update stuff"
- ❌ "Fix"

## Command Coverage

### Currently Documented

The skill currently covers:
- Authentication (`glab auth`)
- Merge Requests (`glab mr`)
- Issues (`glab issue`)
- CI/CD (`glab ci`, `glab pipeline`)
- Repositories (`glab repo`)
- API (`glab api`)
- Labels, Releases, Snippets, Users

### Could Use More Coverage

Areas that could be expanded:
- `glab duo` - GitLab Duo AI features
- `glab cluster` - Kubernetes cluster management
- `glab iteration` - Iteration management
- `glab stack` - Stack management
- `glab opentofu` - OpenTofu integration
- Advanced API usage patterns
- Enterprise GitLab features

## Documentation Standards

### Command Documentation Template

When documenting a new command, use this structure:

```markdown
### Command Category

#### Basic Usage
```bash
# Simple command
glab command subcommand

# With common flags
glab command subcommand --flag=value
```

#### Common Options
- `--flag1` - Description
- `--flag2` - Description

#### Examples
```bash
# Example 1: Description
glab command subcommand --example

# Example 2: Description
glab command subcommand --another-example
```
```

## Getting Help

If you have questions:
1. Check the [glab official documentation](https://docs.gitlab.com/editor_extensions/gitlab_cli/)
2. Use `glab <command> --help`
3. Review existing skill documentation patterns
4. Open an issue for discussion

## Review Process

Changes will be reviewed for:
- Accuracy of commands and flags
- Clarity of explanations
- Consistency with existing style
- Practical value to users
- Proper testing

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

## Resources

- [glab Official Repository](https://gitlab.com/gitlab-org/cli)
- [GitLab CLI Documentation](https://docs.gitlab.com/editor_extensions/gitlab_cli/)
- [Claude Code Skills Documentation](https://docs.claude.com/en/docs/claude-code/skills)
- [Agent Skills Specification](https://github.com/anthropics/skills/blob/main/agent_skills_spec.md)

Thank you for helping make this skill better! 🚀
