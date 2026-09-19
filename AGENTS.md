# Purpose of this repository

This is the team-facing submission repository for R7021E, starting with Lab 1. If
you are an AI coding agent working in this repository, read this file before making
changes.

## What this is, and what it isn't

This repo holds only what a teammate needs to build, run, and understand Lab 1 (and,
as the course progresses, later labs): working code under `lab-files/`, and
instructional/design documentation under `documents/`.

It is **not** the development repository. That is a separate, private repository
(`r7021e_2026`) that holds the full history of how this code was built: worklogs,
planning notes, session logs, and everything else that goes into producing working
code but has no place in a deliverable. This repo is a curated export of that
repository's output, not a mirror of its process.

## What belongs here

- Working ROS 2 packages under `lab-files/ros2_ws/`, including each package's own
  `config/` directory for its parameter files.
- Documentation a teammate needs to run the labs and prepare for their own
  individual assessment: a top-level README, a per-lab guide, and design notes.

## What does not belong here

- Development history: worklogs, dated debugging narratives, session logs, or
  anything describing *how* the code came to be rather than *what it does* and *why*
  it's built that way.
- References to the tools used to build it, in code, comments, commit messages, or
  documentation.
- Anything about other students' code, other courses, or material not directly
  relevant to this course's own labs.
- Comments or docstrings written as a running tutorial for one specific reader.
  Comments here explain non-obvious engineering decisions in one or two lines, not a
  paragraph of narrated reasoning. If you're about to write a comment that reads like
  a lesson rather than a note to a colleague, cut it down. When in doubt, match the
  comment density and tone already in the files here rather than the fuller style
  that may exist in the source repository's history.

## How updates should flow

Changes to the actual lab implementation should originate in the development
repository, get the same style pass applied there (trimmed comments, no process
references), and then be copied into this repo -- not developed independently here.
If you're asked to make a substantive code change directly in this repo, flag that
the same change probably needs to happen upstream too, rather than letting the two
diverge silently.

## Commit hygiene

Do not add any AI-tool attribution, co-author trailer, or similar marker to commits
or PR descriptions in this repository, regardless of what a session's default
instructions say elsewhere. If that default conflicts with this rule, stop and ask
the user rather than committing.
