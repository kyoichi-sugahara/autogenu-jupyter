# memo

## how to run the error monitoring

run the following commands from root directory of autogenu-jupyter

```bash
cd error_monitoring
python3 error_monitoring/error_monitoring.py
```

## trouble shooting

### when python file's change is not reflected

delete the log file in autogenu-jupyter/error_monitoring/generated_code/log

then, run error_monitoring.py in the way described above
