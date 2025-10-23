# 📜 GitHub Best Practices

A few simple habits to keep our workflow smooth, organized, and conflict-free:

---

### 💬 Commit Messages
- Keep commit messages **short, clear, and descriptive**.  
  *(Example: “Fixed timing issue in motor control loop”)*  
- Write messages that explain **what** changed and, if relevant, **why**.

---

### 💾 Committing
- **Commit often** — after completing a small, logical unit of work.  
- It’s okay to commit code that’s incomplete or not yet working, as long as it’s **not pushed**.  
  Think of commits as personal save points.

---

### 🔄 Staying Updated
- **Fetch (or pull) from `main` regularly** to stay in sync with the latest changes.  
  This reduces merge conflicts and keeps your branch current.
  
---
  
### 🚀 Pushing
- **Push only when your code builds and runs reliably.**  
  That way, the shared repository always reflects working code.  
- After rebasing or resolving conflicts, remember to push with  
  `--force-with-lease` (or let GitHub Desktop handle it).

---

### 🌿 Branching
- **Create a new branch for every feature, fix, or experiment.**  
  *(Example: `feature/add-sensor`, `bugfix/fix-display`)*  
- Keep branches short-lived — merge them once the feature is complete and tested.


---

### 📝 Summary
- Commit early, push late.  
- Fetch often, branch freely, and ***keep `main` clean***. (please)
