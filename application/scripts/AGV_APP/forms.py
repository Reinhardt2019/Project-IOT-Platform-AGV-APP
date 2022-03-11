"""Sign-up & log-in forms."""
from flask_wtf import FlaskForm
from wtforms import StringField, PasswordField, SubmitField
from wtforms.validators import (
    DataRequired,
    EqualTo,
    Length,
)


class SignupForm(FlaskForm):
    """User Sign-up Form."""
    username = StringField(
        'Username',
        validators=[DataRequired()]
    )
    password = PasswordField(
        'Password',
        validators=[
            DataRequired(),
            Length(min=6, message='Select a stronger password.')
        ]
    )
    confirm = PasswordField(
        'Confirm Your Password',
        validators=[
            DataRequired(),
            EqualTo('password', message='Passwords must match.')
        ]
    )
    position = StringField(
        'Position',
        validators=[DataRequired()]
    )
    # TODO: add user type

    submit = SubmitField('Register')


class LoginForm(FlaskForm):
    """User Log-in Form."""
    username = StringField(
        'Username',
        validators=[
            DataRequired(),
        ]
    )
    password = PasswordField('Password', validators=[DataRequired()])
    # TODO: add user type

    submit = SubmitField('Log In')


class ForgotPasswordForm(FlaskForm):
    """User Log-in Form."""
    username = StringField(
        'Username',
        validators=[
            DataRequired(),
        ]
    )
    password = PasswordField(
        'Password',
        validators=[
            DataRequired(),
            Length(min=6, message='Select a stronger password.')
        ]
    )
    confirm = PasswordField(
        'Confirm Your Password',
        validators=[
            DataRequired(),
            EqualTo('password', message='Passwords must match.')
        ]
    )

    submit = SubmitField('Reset')


class OrderForm(FlaskForm):
    """User Order Form."""
    merchandise_name = StringField(
        'Merchandise Name',
        validators=[DataRequired()]
    )
    submit = SubmitField('Order')


class ConfirmForm(FlaskForm):
    """User Order Form."""
    merchandise_name = StringField(
        'Merchandise Name',
        validators=[DataRequired()]
    )
    submit = SubmitField('Confirm')
