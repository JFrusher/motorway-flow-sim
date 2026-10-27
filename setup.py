from setuptools import find_packages, setup

setup(
    name="motorway-flow-sim",
    version="0.1.0",
    description="Microscopic motorway traffic simulation: IDM, MOBIL and lorry slipstream effects",
    author="Jacob Frusher",
    url="https://github.com/JFrusher/motorway-flow-sim",
    license="MIT",
    packages=find_packages(include=["motorway", "motorway.*"]),
    python_requires=">=3.10",
    install_requires=["numpy>=1.24", "pandas>=2.0", "matplotlib>=3.7", "pyyaml>=6.0"],
    extras_require={"render": ["pygame-ce>=2.4"], "dev": ["pytest", "flake8", "black"]},
)
