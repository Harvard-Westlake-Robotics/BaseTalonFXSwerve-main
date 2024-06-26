package frc.lib.util;

import java.util.ArrayList;
import java.util.List;

// A class that represents a simplistic promise for asynchronous operations.
public class SimplePromise extends Promise {

    private boolean resolved = false; // Flag to check if the promise has been resolved.

    // A list to hold the lambda expressions (callbacks) that should run after the
    // promise is resolved.
    private List<Lambda> callbacks = new ArrayList<>();

    // Checks if the promise has been resolved.
    public boolean isResolved() {
        return resolved;
    }

    // Adds a callback to be executed when the promise is resolved.
    public void i_then(Lambda callback) {
        if (resolved) {
            callback.run();
        } else {
            callbacks.add(callback);
        }
    }

    // Resolves the promise, running any callbacks that have been added.
    public void resolve() {
        if (!resolved) {
            resolved = true;
            for (Lambda callback : callbacks) {
                callback.run();
            }
        }
    }
}