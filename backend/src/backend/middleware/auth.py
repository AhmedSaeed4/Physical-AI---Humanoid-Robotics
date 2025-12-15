from fastapi import HTTPException, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import jwt, JWTError
from pydantic import BaseModel
from typing import Optional
import os
from dotenv import load_dotenv

load_dotenv()

# JWT Configuration
JWT_SECRET = os.getenv("JWT_SECRET")
if not JWT_SECRET:
    raise ValueError("JWT_SECRET environment variable is not set")

ALGORITHM = "HS256"

class UserPayload(BaseModel):
    """Structure for user payload in JWT token"""
    sub: str  # user ID
    email: str
    name: Optional[str] = None
    educationLevel: Optional[str] = None
    programmingExperience: Optional[str] = None
    roboticsBackground: Optional[str] = None
    softwareBackground: Optional[str] = None
    hardwareBackground: Optional[str] = None
    exp: int  # expiration timestamp

class JWTBearer(HTTPBearer):
    """Custom JWT Bearer authentication class"""

    def __init__(self, auto_error: bool = True):
        super(JWTBearer, self).__init__(auto_error=auto_error)

    async def __call__(self, request: Request):
        credentials: HTTPAuthorizationCredentials = await super(JWTBearer, self).__call__(request)

        if credentials:
            if not credentials.scheme == "Bearer":
                raise HTTPException(
                    status_code=401,
                    detail="Invalid authentication scheme."
                )
            token = credentials.credentials
            user_data = self.verify_jwt(token)
            if not user_data:
                raise HTTPException(
                    status_code=401,
                    detail="Invalid token or expired token."
                )
            # Add user data to request for use in endpoints
            request.state.user = user_data
            return token
        else:
            raise HTTPException(
                status_code=403,
                detail="Authentication credentials were not provided."
            )

    def verify_jwt(self, jwt_token: str) -> Optional[UserPayload]:
        """Verify JWT token and return user data if valid"""
        try:
            payload = jwt.decode(jwt_token, JWT_SECRET, algorithms=[ALGORITHM])
            user_data = UserPayload(**payload)
            return user_data
        except JWTError:
            return None

def get_current_user(request: Request) -> UserPayload:
    """Get current user from request state (after JWT verification)"""
    return getattr(request.state, 'user', None)