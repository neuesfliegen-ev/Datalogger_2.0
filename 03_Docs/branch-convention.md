# 🌳 Branch Naming & Workflow Convention

To keep our collaboration organized and avoid conflicts, please follow these guidelines when creating and managing branches.

> See also: [📜 GitHub Best Practices](\docs\github-best-practices.md)

---

### 🌿 1. Creating Branches
- **Create a new branch** for every new feature, bug fix, or major change.  
- Ideally, **one person works on one branch** at a time.  
  This keeps history clean and prevents conflicting edits.

---

### 🏷️ 2. Naming Conventions
- Use **descriptive, lowercase, hyphen-separated names** that reflect the purpose of the branch.  
  Examples:
  - `feature/implement-oled`
  - `bugfix/fix-timing-loop`
  - `experiment/new-gps-module`

This makes it easy for everyone to understand what each branch is for at a glance.

---

### 🔄 3. Keeping Your Branch Up to Date
- If `main` gets updated while you’re working, **rebase your branch** onto the latest `main`.  
  This ensures your work includes the newest changes and reduces merge conflicts later.  
  *(Command line: `git fetch origin && git rebase origin/main` or let GitHub Desktop handle it 🤷🏼)*.

---

### 🚀 4. Opening Pull Requests
- When your code is ready and tested, **push your branch** to GitHub and open a **Pull Request (PR)**.  
- Request at least one teammate to review before merging into `main`.

---

### 🧹 5. Cleaning Up
- After your PR is merged, delete your branch locally. Your branch on GitHub will be deleted automatically.
- This keeps the repository tidy and prevents confusion.

---

### 🤓☝🏼 6. Exceptions
- If you’re working on something that **no one else will touch** (e.g., a personal test file or documentation),  
  you don’t need to strictly follow this branching convention.  
- Just make sure your work doesn’t interfere with shared code.

---

### ⚠️ Important Notes

- Never rebase or force-push shared branches that others might be using.
Only rebase your own feature branches.

- When in doubt, **ask before merging** — better a small delay than broken code!