"""Routes for user authentication."""
from flask import Blueprint, render_template
from . import login_manager

# Blueprint Configuration
auth_bp = Blueprint(
    'auth_bp', __name__,
    template_folder='templates',
    static_folder='static'
)


@auth_bp.route('/login', methods=['GET', 'POST'])
def login():
    # Login route logic goes here
    return render_template('')


@auth_bp.route('/signup', methods=['GET', 'POST'])
def signup():
    # Signup logic goes here
    return render_template('')
