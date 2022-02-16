from setuptools import setup

setup(
    name='AGV-APP',
    packages=['AGV-APP'],
    include_package_data=True,
    install_requires=[
        'flask',
        'SQLAlchemy',
        'dotenv',
    ],
)