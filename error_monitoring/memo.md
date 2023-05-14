# memo

## how to run the error monitoring

run the following commands from root directory of autogenu-jupyter

```bash
python3 error_monitoring/error_monitoring.py
```

## trouble shooting

### when python file's change is not reflected

The python module `autogenu` can be instatlled by running
```
python3 -m pip install setuptools
python3 -m pip install .
```
at the project root directory of `autogenu-jupyter`.

then, run error_monitoring.py in the way described above
