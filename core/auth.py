#
#  Copyright (c) 2025 Composiv.ai
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# Licensed under the  Eclipse Public License v2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v20.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai - initial API and implementation
#
#

import os
import time
import threading
import requests
from pathlib import Path
from typing import Optional
from dataclasses import dataclass

try:
    from dotenv import load_dotenv
    # Load .env file from project root (search up from current file)
    env_path = Path(__file__).resolve().parents[4] / '.env'
    if env_path.exists():
        load_dotenv(env_path)
except ImportError:
    # python-dotenv not installed, rely on system environment variables
    pass


@dataclass
class JWTConfig:
    """Configuration for JWT authentication."""
    auth_url: str
    client_id: str
    client_secret: str
    username: str
    password: str

    @classmethod
    def from_env(cls) -> Optional['JWTConfig']:
        """Load JWT configuration from environment variables."""
        auth_url = os.environ.get('AUTH_URL', '')
        client_id = os.environ.get('AUTH_CLIENT_ID', '')
        client_secret = os.environ.get('AUTH_CLIENT_SECRET', '')
        username = os.environ.get('AUTH_USERNAME', '')
        password = os.environ.get('AUTH_PASSWORD', '')

        if not all([auth_url, client_id, client_secret, username, password]):
            return None

        return cls(
            auth_url=auth_url,
            client_id=client_id,
            client_secret=client_secret,
            username=username,
            password=password
        )

    @classmethod
    def from_params(cls, auth_url: str, client_id: str, client_secret: str,
                    username: str, password: str) -> Optional['JWTConfig']:
        """Create JWT configuration from parameters, falling back to env vars."""
        # Use provided values, fall back to environment variables if empty
        auth_url = auth_url or os.environ.get('AUTH_URL', '')
        client_id = client_id or os.environ.get('AUTH_CLIENT_ID', '')
        client_secret = client_secret or os.environ.get('AUTH_CLIENT_SECRET', '')
        username = username or os.environ.get('AUTH_USERNAME', '')
        password = password or os.environ.get('AUTH_PASSWORD', '')

        if not all([auth_url, client_id, client_secret, username, password]):
            return None

        return cls(
            auth_url=auth_url,
            client_id=client_id,
            client_secret=client_secret,
            username=username,
            password=password
        )


class JWTAuthenticator:
    """
    Handles JWT authentication with Keycloak OpenID Connect.

    Features:
    - Token caching to avoid unnecessary auth requests
    - Proactive token refresh at 80% of token lifetime
    - Thread-safe token management
    """

    def __init__(self, config: JWTConfig, refresh_threshold: float = 0.8):
        """
        Initialize the JWT authenticator.

        Args:
            config: JWT configuration containing auth endpoint and credentials
            refresh_threshold: Refresh token when this fraction of lifetime has passed (default: 0.8)
        """
        self._config = config
        self._refresh_threshold = refresh_threshold
        self._token: Optional[str] = None
        self._token_expiry: float = 0
        self._lock = threading.Lock()

    def get_auth_headers(self) -> dict:
        """
        Get authorization headers with a valid JWT token.

        Returns:
            dict: Headers containing the Authorization Bearer token
        """
        token = self._get_valid_token()
        return {"Authorization": f"Bearer {token}"}

    def _get_valid_token(self) -> str:
        """Get a valid token, refreshing if necessary."""
        with self._lock:
            if self._should_refresh():
                self._refresh_token()
            return self._token

    def _should_refresh(self) -> bool:
        """Check if token needs to be refreshed."""
        if self._token is None:
            return True

        current_time = time.time()
        return current_time >= self._token_expiry

    def _refresh_token(self) -> None:
        """Fetch a new JWT token from the auth server."""
        payload = {
            'grant_type': 'password',
            'client_id': self._config.client_id,
            'client_secret': self._config.client_secret,
            'username': self._config.username,
            'password': self._config.password
        }

        headers = {'Content-Type': 'application/x-www-form-urlencoded'}

        response = requests.post(
            self._config.auth_url,
            data=payload,
            headers=headers,
            timeout=30
        )
        response.raise_for_status()

        token_data = response.json()
        self._token = token_data['access_token']

        # Calculate expiry time with refresh threshold
        expires_in = token_data.get('expires_in', 300)  # Default 5 minutes
        refresh_at = expires_in * self._refresh_threshold
        self._token_expiry = time.time() + refresh_at

    def clear_token(self) -> None:
        """Clear the cached token, forcing a refresh on next request."""
        with self._lock:
            self._token = None
            self._token_expiry = 0
