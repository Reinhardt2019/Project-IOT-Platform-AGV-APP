"""Database models."""
from . import db
from flask_security import UserMixin, RoleMixin
from werkzeug.security import generate_password_hash, check_password_hash


class RolesUsers(db.Model):
    __tablename__ = 'roles_users'
    id = db.Column(db.Integer(), primary_key=True)
    user_id = db.Column('user_id', db.Integer(), db.ForeignKey('user.id'))
    role_id = db.Column('role_id', db.Integer(), db.ForeignKey('role.id'))


class User(UserMixin, db.Model):
    """User account model."""

    __tablename__ = 'user'
    id = db.Column(
        db.Integer,
        primary_key=True
    )
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
