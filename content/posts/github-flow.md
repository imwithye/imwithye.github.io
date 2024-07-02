---
title: "Transitioning from GitFlow to GitHub PR Workflow: A Journey in Streamlining Our Development Process"
date: 2024-07-02
tags: ["GitHub"]
---

# Transitioning from GitFlow to GitHub PR Workflow: A Journey in Streamlining Our Development Process

## Introduction

In our company, we originally adopted the GitFlow model for our development workflow. This model served us well, facilitating a structured approach to feature development, release management, and hotfixes. However, with the introduction of GitHub and its powerful pull request (PR) workflow, we decided to transition to a more integrated system. This blog post outlines our journey, the challenges we faced, and the solutions we implemented to streamline our development process.

## The GitFlow Model

Under the GitFlow model, our development process was as follows:

1. **Feature Development**: All features were checked out from the `staging` branch.
2. **Merging**: Features were merged back into `staging`.
3. **Release**: When ready to release, `staging` was merged into `main`, triggering a production deployment.
4. **Hotfixes**: If a bug was found in `main`, a hotfix branch was created from `main`, fixed, and then merged back into both `staging` and `main`.

This workflow ensured stability and control, as any code merged into `main` was deemed ready for production. However, with the shift to using GitHub, we sought to leverage its PR and code review capabilities to enhance our process further.

## Transition to GitHub PR Workflow

With GitHub, we restructured our workflow to utilize PRs more effectively:

1. **Development Branch**: We set `main` as our development branch.
2. **Feature Branches**: Features were checked out from `main` and developed.
3. **PRs to Main**: Developers created PRs to merge features back into `main`.
4. **Automatic Deployment**: Upon PR approval, `main` automatically deployed to `staging`.
5. **Tagging for Production**: Once a tag was created on `main`, it triggered an automatic deployment to production.

This new workflow integrated GitHub's PR and code review features seamlessly. However, it introduced a significant challenge: handling hotfixes. Since `main` was now our development branch, any hotfix merged into `main` could inadvertently include unstable code, potentially affecting production.

## Refining the Workflow

To address this issue, we made further modifications to our model while retaining the benefits of GitHub PRs:

1. **Branch Structure**: We retained both `main` and `staging` branches.
2. **PR to Staging**: All developers were required to PR their changes to `staging`.
3. **Releases**: When ready to release, a PR from `staging` to `main` was created.
4. **Hotfixes**: In case of a hotfix, the fix was branched from `main`, PR'd to both `main` and `staging`.

Although effective, this approach necessitated creating two PRs for each hotfix – one to `main` and one to `staging`. To simplify this process, we leveraged GitHub Actions to automate the synchronization of hotfixes across branches.

## Automation with GitHub Actions

We created a GitHub Action using a bot account to automate our hotfix workflow:

1. **Hotfix PR**: When a hotfix PR to `main` is accepted, the action checks out `main` to a `patch` branch.
2. **Automatic PR**: The `patch` branch automatically creates a PR to `staging`.
3. **Conflict Resolution**: In case of conflicts, the bot's PR allows manual conflict resolution on the `patch` branch before updating the PR to `staging`.

This automation ensures that our hotfixes are focused on `main`, with `staging` being automatically updated via a synchronized `patch` branch. This approach maintains the integrity of our branches and simplifies the workflow for our developers.

## GitHub Action Script

Below is the GitHub Action script we used to implement this workflow:

```yaml
name: Pull Main

on:
  push:
    branches:
      - main

jobs:
  merge:
    if: github.event.pull_request == null || github.event.pull_request.head.ref != 'staging'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive
          token: ${{ secrets.PAT_TOKEN }}
      - name: Push a new branch
        id: push_branch
        run: |
          git config --global user.name "${{ vars.INFR_AI_BOT_NAME }}"
          git config --global user.email "${{ vars.INFR_AI_BOT_EMAIL }}"
          echo "branch_name=pull-main-$GITHUB_SHA" >> "$GITHUB_OUTPUT"
          git checkout -b pull-main-$GITHUB_SHA
          git push origin pull-main-$GITHUB_SHA
      - uses: actions/github-script@v7
        env:
          WORKING_BRANCH: ${{ steps.push_branch.outputs.branch_name }}
        with:
          github-token: ${{ secrets.PAT_TOKEN }}
          script: |
            github.rest.pulls.create({
              owner: context.repo.owner,
              repo: context.repo.repo,
              title: 'chore: pull main branch into staging',
              head: process.env.WORKING_BRANCH,
              base: 'staging',
              body: 'Automatically generated pull request to merge changes from the main branch into the staging branch.'
            })
```

## Conclusion

By transitioning from GitFlow to GitHub's PR workflow, we enhanced our development process, integrating modern code review practices. The automation with GitHub Actions further streamlined our workflow, allowing us to maintain stability and control across our branches. This journey underscores the importance of continually evolving our processes to leverage new tools and technologies effectively.
