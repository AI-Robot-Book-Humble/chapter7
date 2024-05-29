# Sample State Machine with FlexBE

TODO


## Example States in `sample_sm_flexbe_states`

Packages providing FlexBE states are identified by an export tag in the `package.xml`:

```xml
  <export>
      <flexbe_states />
      <build_type>ament_cmake</build_type>
  </export>
```

* `example_state.py `
  * Example state implementation with extra console logging to show the state life cycle.

* `example_action_state.py`


## Example Behaviors in `sample_sm_flexbe_behaviors`

Packages providing FlexBE behaviors are identified by an export tag in the `package.xml`:

```xml
  <export>
      <flexbe_behaviors />
      <build_type>ament_cmake</build_type>
  </export>
```

  * `example_behavior_sm.py`
    * Most basic example state machine

  * `example_action_behavior_sm.py` 
    * Uses the `ExampleActionState` with the standard action tutorials 
