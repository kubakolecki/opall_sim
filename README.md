# Running simulator
General: Scripts were tested using Python 3.12 and virtual environment.  
## Running simulator using virtual environment in Windows and Power Shell:  
1. Go to the top directory of the package (the one where the requirements.txt file resides).  
2. In Windows Power Shell verify the Python version:
```bash
python --version
```
3. Create virtual environment:
```bash
python -m venv venv
```
4. Activate virtual environment:
```bash
venv\Scripts\Activate.ps1
```
5. Install required packages:
```bash
pip install -r requirements.txt
```
(You might want to upgrade pip first). This step you can execute only once, when you are setting up virtual environment for the first time.

6. Test if the example runs, by runnig sample Power Shell script:
```bash
scripts\run_simulation_01.ps1
```
7. If there are no Python errors, everything should work fine.
8. Don't forget to deactivate virtial environment:
```bash
deactivate
```

## Running simulator using virtual environment in Linux:
1. Go to the top directory of the package (the one where the requirements.txt file resides).
2. In Linux terminal verify the Python version:
```bash
python3 --version
```

3. Create virtual environment:
```bash
python3 -m venv venv
```

4. Activate virtual environment:
```bash
source venv/bin/activate
```

5. Install required packages:
```bash
pip install -r requirements.txt
```
(You might want to upgrade pip first). This step you can execute only once, when you are setting up virtual environment for the first time.

6. Test if the example runs, by runnig sample bash script:
```bash
bash scripts/run_simulation_01.sh
```

7. If there are no Python errors, everything should work fine.
    
8. Don't forget to deactivate virtial environment:
```bash
deactivate
```


Simulation results should be generated in sample_data/simulation_01/simulation_output. You can run simulator for sample_data/simulation_02 in a similar way.
