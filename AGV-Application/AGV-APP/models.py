"""Database models."""
from . import db
from flask_security import UserMixin, RoleMixin
from sqlalchemy_utils import UUIDType
import uuid


class User(UserMixin, db.Model):
    """User account model."""

    __tablename__ = 'user'
    # an unique id generate by the application
    id = db.Column(
        UUIDType(binary=False),
        default=uuid.uuid4,
        primary_key=True
    )
    # user defined username, unique
    username = db.Column(
        db.String(100),
        nullable=False,
        unique=False
    )
    password = db.Column(
        db.String(200),
        primary_key=False,
        unique=False,
        nullable=False
    )
    # TODO: change this field to be foreign key
    position = db.Column(
        db.String(200),
        primary_key=False,
        unique=True,
        nullable=False
    )
    active = db.Column(
        db.Boolean
    )
    confirmed_at = db.Column(
        db.DateTime
    )
    roles = db.relationship(
        'Role',
        secondary='roles_users',
        backref=db.backref('users', lazy='dynamic')
    )


class Role(RoleMixin, db.Model):
    __tablename__ = 'role'
    id = db.Column(db.Integer(), primary_key=True)
    name = db.Column(db.String(80), unique=True)
    description = db.Column(db.String(255))