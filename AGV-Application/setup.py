from setuptools import setup

# TODO: complete install_requires
setup(
    name='AGV-Application',
    packages=['AGV-Application'],
    include_package_data=True,
    install_requires=[
        'flask',
        'SQLAlchemy',
        'dotenv',
    ],
)