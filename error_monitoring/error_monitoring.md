# autogenu design

TODO

## autogenu Requirement

TODO

## Generating autogenu Candidate Path

TODO

### Preparation phase

TODO

### Flow chart

The following charts illustrate the flow of the OCP

```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title OCP
start
:**INPUT** hoge;

:idx = 0;


if( residual < TRUE?) then (YES)
else (NO)
endif
:++idx;


:**RETURN** hoge;

stop
@enduml

```


## Parameters

### Essential parameters

The following parameters are configurable in `hoge`.

| Name                        | Unit | Type    | Description                             | Default value |
| :-------------------------- | ---- | ------- | --------------------------------------- | ------------- |
| `hoge` | [-]  | boolean | Hoge               | true          |

### Debug

The following parameters are configurable

| Name                   | Unit | Type    | Description                  | Default value |
| :--------------------- | ---- | ------- | ---------------------------- | ------------- |
| `debug_param` | [-]  | boolean | Flag to output debug data | false         |

## Future Improvements

- 

## References

TODO
