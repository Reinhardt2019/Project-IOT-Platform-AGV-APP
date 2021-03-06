"""Routes for user authentication."""
from flask import Blueprint, redirect, render_template, flash, request, session, url_for
from . import login_manager, user_datastore, order_datastore
from flask_login import login_required, logout_user, current_user
from .forms import SignupForm, LoginForm, ForgotPasswordForm
from .models import db, User
from flask_security.utils import hash_password, login_user, verify_and_update_password
import uuid
import datetime


# Blueprint Configuration
auth_bp = Blueprint(
    'auth_bp', __name__,
    template_folder='templates',
    static_folder='static'
)


@auth_bp.route('/signup', methods=['GET', 'POST'])
def signup():
    # Signup route logic goes here
    form = SignupForm()
    # User sign-up logic
    if form.validate_on_submit():
        # check for username duplication
        existing_user = user_datastore.find_user(username=request.form.get('username'))
        # create and register new user
        if existing_user is None:
            session['user_id'] = uuid.uuid4()
            new_user = user_datastore.create_user(
                id=session['user_id'],
                username=request.form.get('username'),
                password=hash_password(request.form.get('password')),
                position_id=request.form.get('position'),
                confirmed_at=datetime.date.today()
            )
            order_datastore.create_user_order_store(new_user)
            db.session.commit()  # create new user
            # login new user
            login_user(new_user)
            # TODO: login redirection filtered by user type
            return redirect(url_for('main_bp.dashboard'))
        # flash('username is already occupied.') TODO: flash does not work
        return redirect(url_for('auth_bp.fail'))
    return render_template('signup.html', form=form)


@auth_bp.route('/fail')
def fail():
    return render_template('fail.html')


@auth_bp.route('/login', methods=['GET', 'POST'])
def login():
    # Login logic
    """
      Log-in page for registered users.

      GET requests serve Log-in page.
      POST requests validate and redirect user to dashboard.
      """
    # Bypass if user is logged in
    if current_user.is_authenticated:
        return redirect(url_for('main_bp.dashboard'))

    form = LoginForm()
    # Validate login attempt
    if form.validate_on_submit():
        user = user_datastore.find_user(username=form.username.data)
        if user and verify_and_update_password(password=form.password.data, user=user):
            # Login current user
            login_user(user)
            session['user_id'] = user.id
            next_page = request.args.get('next')
            return redirect(next_page or url_for('main_bp.dashboard'))
        flash('Invalid username/password combination')
        return redirect(url_for('auth_bp.login'))
    return render_template('login.html', form=form)


@auth_bp.route('/forgot_password', methods=['GET', 'POST'])
def forgot_password():
    if current_user.is_authenticated:
        return redirect(url_for('main_bp.dashboard'))

    form = ForgotPasswordForm()
    # Validate login attempt
    if form.validate_on_submit():
        user = user_datastore.find_user(username=form.username.data)
        user.password = hash_password(request.form.get('password'))
        user_datastore.put(user)
        db.session.commit()
        return redirect(url_for('main_bp.dashboard'))
    return render_template('forgot_password.html', form=form)


@login_manager.user_loader
def load_user(user_id):
    """Check if user is logged-in on every page load."""
    if user_id is not None:
        return User.query.get(user_id)
    return None


@login_manager.unauthorized_handler
def unauthorized():
    """Redirect unauthorized users to Login page."""
    flash('You must be logged in to view that page.')
    return redirect(url_for('auth_bp.login'))
