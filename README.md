# ASR-Software


## Entorno de simulacion y robot real con Kobuki

Ver con extremo cuidado  [kobuki_robot/kobuki/README.md](kobuki_robot/kobuki/README.md)

## Creación de un entorno virtual para Python

```
python3 -m venv venv_asr
echo -e '\n# Add venv_asr site-packages to PYTHONPATH\nVENV_SITE_PACKAGES="$VIRTUAL_ENV/lib/python3.12/site-packages"\nif [ -z "$PYTHONPATH" ]; then\n    export PYTHONPATH="$VENV_SITE_PACKAGES"\nelse\n    export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"\nfi' >> venv_asr/bin/activate 
cd venv_asr/
touch COLCON_IGNORE
cd ..
source venv_asr/bin/activate
```
