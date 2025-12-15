import express from 'express';
import jwt from 'jsonwebtoken';
import { auth } from '../auth';

const router = express.Router();

// JWT token generation middleware
const generateJWTToken = (req: express.Request, res: express.Response, next: express.NextFunction) => {
  // This middleware will be called after Better Auth processes the request
  // We'll add JWT token to the response for backend use
  const originalJson = res.json;

  res.json = function(data: any) {
    // If this is a successful auth response (signup/login), add JWT token
    if (data && data.user && !data.error) {
      const jwtSecret = process.env.JWT_SECRET;
      if (!jwtSecret) {
        console.error('JWT_SECRET not configured');
        return originalJson.call(this, data);
      }

      // Create JWT token with user data for backend
      const tokenPayload = {
        sub: data.user.id,
        email: data.user.email,
        name: data.user.name,
        educationLevel: data.user.educationLevel || 'Not specified',
        programmingExperience: data.user.programmingExperience || 'Not specified',
        roboticsBackground: data.user.roboticsBackground || 'Not specified',
        softwareBackground: data.user.softwareBackground || 'Not specified',
        hardwareBackground: data.user.hardwareBackground || 'Not specified'
      };

      const token = jwt.sign(tokenPayload, jwtSecret, { expiresIn: '24h' });

      // Add JWT token to response
      data.jwtToken = token;
    }

    return originalJson.call(this, data);
  };

  next();
};

// Apply JWT token generation to auth routes
router.use(generateJWTToken);

// Better Auth handles these routes automatically via middleware in index.ts
// This router is mainly for custom logic and JWT token generation

export default router;