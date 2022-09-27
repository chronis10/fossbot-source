## Installation Instructions



### Installation options 

```bash
# For Coppelia
--install-option="--platform=sim"

# For Real
--install-option="--platform=real"

# For Dummy
#without --install-option
```

### For local build and installation

```bash
python setup.py build
pip install . --install-option="--platform=sim"
```

### Installation from the Github Repository

```bash
pip install git+https://github.com/chronis10/fossbot_source.git@fossbot_as_lib --install-option="--platform=sim"
```