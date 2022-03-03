"""Database models."""
from . import db
from flask_security import UserMixin, RoleMixin
from sqlalchemy_utils import UUIDType
import uuid
import enum


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
    position = db.Column(
        db.String(200),
        db.ForeignKey('position.id'),
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


class Position(db.Model):
    __tablename__ = 'position'
    id = db.Column(db.Integer(), primary_key=True)
    x = db.Column(db.Float())
    y = db.Column(db.Float())
    z = db.Column(db.Float())
    w = db.Column(db.Float())


class Merchandise(db.Model):
    __tablename__ = 'merchandise'
    id = db.Column(UUIDType(binary=False),
                   default=uuid.uuid4,
                   primary_key=True)
    name = db.Column(db.String(80), unique=True)
    type = db.Column(db.String(80))
    description = db.Column(db.String(255))


class OrderStatus(enum.Enum):
    PENDING = 1
    DELIVERING = 2
    COMPLETED = 3
    FAILED = 4


class Order(db.Model):
    __tablename__ = 'order'
    id = db.Column(UUIDType(binary=False),
                   default=uuid.uuid4,
                   primary_key=True)
    merchandise_id = db.Column(UUIDType(binary=False),
                               db.ForeignKey('merchandise.id'),
                               default=uuid.uuid4)
    order_status = db.Column(db.Enum(OrderStatus))
    ordered_time = db.Column(db.DateTime)
    estimated_time = db.Column(db.Time)
