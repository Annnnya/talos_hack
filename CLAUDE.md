# Claude Code instructions

The shared, tool-agnostic rulebook lives in `AGENTS.md`. Claude Code pulls it in via the import
below. **Edit shared rules in `AGENTS.md`, not here.** This file holds only Claude-Code-specific
notes.

@AGENTS.md

## Claude-Code-specific notes

- **Sub-agents and worktrees:** when spawning sub-agents that edit code, pass
  `isolation: "worktree"` to the Agent tool so each one gets its own worktree.
- **Hardware:** never run a `talos-servo` command (or any script that opens a serial port) unless
  the user asks for it in the current session.
- **Personal notes:** put personal, non-shared instructions in `CLAUDE.local.md` (gitignored).
