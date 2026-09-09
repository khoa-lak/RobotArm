"""
Undo / Redo Manager for RobotArm HMI.
Tracks user actions and allows bidirectional undo/redo operations with state snapshots.
"""

from typing import Callable, Any, Optional, List


class Action:
    """Represents a single undoable/redoable action."""
    def __init__(self, description: str, undo_fn: Callable[[], Any], redo_fn: Callable[[], Any]):
        self.description = description
        self.undo_fn = undo_fn
        self.redo_fn = redo_fn

    def undo(self):
        if callable(self.undo_fn):
            self.undo_fn()

    def redo(self):
        if callable(self.redo_fn):
            self.redo_fn()

    def __repr__(self):
        return f"<Action: {self.description}>"


class UndoRedoManager:
    """Manages undo and redo stacks with a maximum history depth."""
    def __init__(self, max_history: int = 50, on_change_callback: Optional[Callable[[], Any]] = None):
        self.max_history = max_history
        self.undo_stack: List[Action] = []
        self.redo_stack: List[Action] = []
        self.on_change_callback = on_change_callback
        self.is_executing = False

    def push(self, description: str, undo_fn: Callable[[], Any], redo_fn: Callable[[], Any]):
        """Pushes a new action to the undo stack and clears the redo stack."""
        if self.is_executing:
            return

        action = Action(description, undo_fn, redo_fn)
        self.undo_stack.append(action)
        if len(self.undo_stack) > self.max_history:
            self.undo_stack.pop(0)

        self.redo_stack.clear()
        self._notify()

    def undo(self) -> Optional[str]:
        """Undoes the most recent action. Returns description of undone action, or None."""
        if not self.can_undo() or self.is_executing:
            return None

        action = self.undo_stack.pop()
        self.is_executing = True
        try:
            action.undo()
            self.redo_stack.append(action)
        finally:
            self.is_executing = False

        self._notify()
        return action.description

    def redo(self) -> Optional[str]:
        """Redoes the most recently undone action. Returns description of redone action, or None."""
        if not self.can_redo() or self.is_executing:
            return None

        action = self.redo_stack.pop()
        self.is_executing = True
        try:
            action.redo()
            self.undo_stack.append(action)
        finally:
            self.is_executing = False

        self._notify()
        return action.description

    def can_undo(self) -> bool:
        """Returns True if there is at least one action that can be undone."""
        return len(self.undo_stack) > 0

    def can_redo(self) -> bool:
        """Returns True if there is at least one action that can be redone."""
        return len(self.redo_stack) > 0

    def get_undo_description(self) -> str:
        """Returns description of the next action to undo."""
        if self.undo_stack:
            return self.undo_stack[-1].description
        return ""

    def get_redo_description(self) -> str:
        """Returns description of the next action to redo."""
        if self.redo_stack:
            return self.redo_stack[-1].description
        return ""

    def clear(self):
        """Clears both undo and redo stacks."""
        self.undo_stack.clear()
        self.redo_stack.clear()
        self._notify()

    def _notify(self):
        if callable(self.on_change_callback):
            try:
                self.on_change_callback()
            except Exception as e:
                print(f"[ERROR] UndoRedoManager callback failed: {e}")
