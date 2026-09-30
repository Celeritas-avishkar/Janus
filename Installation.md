# <ins>**Windows Installation :**</ins>

## **Step 1 - Install Git**

Install Git for Windows if Git is not already installed.

After installation, open PowerShell, Command Prompt, or Git Bash and check:

```bash
git --version
```

You should see a Git version number.

## **Step 2 - Install Python**

Install Python 3 for Windows.

Check the installation:

```bash
python --version
```

If python is not recognized, try:

```bash
py --version
```

## **Step 3 - Clone the repository**

From the folder where you want the project:

```bash
git clone https://github.com/Celeritas-avishkar/Janus
cd <YOUR-REPOSITORY-FOLDER>
```

## **Step 4 - Create a virtual environment**

Recommended:

```bash
python -m venv .venv
```

If your machine uses the Python launcher:

```bash
py -m venv .venv
```

Activate it:

```bash
.venv\Scripts\Activate.ps1
```

If PowerShell blocks activation, you can either use Command Prompt:

```bash
.venv\Scripts\activate.bat
```

or run Python directly from the environment.

## **Step 5 - Install dependencies**

With the virtual environment active:

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

If requirements.txt is not present, install Pygame directly:

```bash
python -m pip install pygame
```

## **Step-6 Verify the installation**

Before running the full simulator, check Pygame:

```bash
python -c "import pygame; print(pygame.version.ver)"
```

Then check the program syntax without starting the GUI:

```bash
python -m py_compile main.py
```

**If there is no output, the syntax check passed.**

## **Step 7 - Run the simulator**

```bash
python main.py
```
---

 # <ins>**macOS installation :**</ins>

## **Step 1 - Install Git**

macOS normally includes Git through the Xcode Command Line Tools.

Check:

```bash
git --version
```

If macOS asks to install the Command Line Tools, accept the installation.

## **Step 2 - Install Python 3**

Check:

```bash
python3 --version
```
> [!IMPORTANT]
> Use Python 3 rather than relying on the old python command.

## **Step 3 - Clone the repository**

```bash
git clone https://github.com/Celeritas-avishkar/Janus
cd <YOUR-REPOSITORY-FOLDER>
```

## **Step 4 - Create a virtual environment**

```bash
python3 -m venv .venv
```

Activate it:

```bash
source .venv/bin/activate
```

Your terminal should now show something similar to:


(.venv)

## **Step 5 - Install dependencies**

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Or, if there is no requirements file:

```bash
python -m pip install pygame
```

## **Step 6 - Run the simulator**

run:

```bash
python main.py
```

## **Verify the installation**

Before running the full simulator, check Pygame:

```bash
python -c "import pygame; print(pygame.version.ver)"
```

Then check the program syntax without starting the GUI:

```bash
python -m py_compile main.py
```

If there is no output, the syntax check passed.

On macOS, use the active virtual environment and run the following commands:

```bash
python -c "import pygame; print(pygame.version.ver)"
python -m py_compile main.py
```
