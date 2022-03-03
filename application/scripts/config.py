"""Flask configuration."""
from os import environ, path
from dotenv import load_dotenv

basedir = path.abspath(path.dirname(__file__))
load_dotenv(path.join(basedir, '.env'))


class Config:
    """Base config."""

    SECRET_KEY = environ.get('SECRET_KEY')
    SECURITY_PASSWORD_SALT=environ.get('SECURITY_PASSWORD_SALT')
    SECURITY_LOGIN_USER_TEMPLATE = environ.get('SECURITY_LOGIN_USER_TEMPLATE')

    # Database
    SQLALCHEMY_DATABASE_URI = environ.get('SQLALCHEMY_DATABASE_URI')
    SQLALCHEMY_ECHO = False
    SQLALCHEMY_TRACK_MODIFICATIONS = False