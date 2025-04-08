Scripts were tested using Python 3.9 and virtual environment.
Running using virtual environment using Windows and Power Shell:  
1. Go to the top directory of the package (the one where the requirements.txt file resides).  
2. In Windows Power Shell check the Python version: `python --version`  
3. Create virtual environment: `python -m venv venv`  
4. Activate virtual environment: `venv\Scripts\Activate.ps1`  
5. Install required packages: `pip install -r requirements.txt` (You might want to upgrade pip first). This step you can execute only once.  
6. Test if the exaple runs, by runnig sample bash script: `run_simulation_01.ps1`  
7. If there are no Python errors, everything should work fine.  
8. Don't forget to deactivate virtial environment: `deactivate`  
