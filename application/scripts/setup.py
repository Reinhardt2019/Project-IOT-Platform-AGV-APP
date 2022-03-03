from setuptools import setup

# TODO: complete install_requires
setup(
    name='scripts',
    packages=['scripts'],
    include_package_data=True,
    install_requires=[
        'flask',
        'SQLAlchemy',
        'dotenv',
    ],
)