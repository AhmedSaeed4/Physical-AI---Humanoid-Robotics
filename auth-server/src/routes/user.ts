import express from 'express';
import { auth } from '../auth';
import { fromNodeHeaders } from 'better-auth/node';

const router = express.Router();

// Middleware to verify user session
const verifyUser = async (req: express.Request, res: express.Response, next: express.NextFunction) => {
  try {
    const session = await auth.api.getSession({
      headers: fromNodeHeaders(req.headers)
    });

    if (!session) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    (req as any).user = session.user;
    (req as any).session = session.session;
    return next();
  } catch (error) {
    console.error('Session verification failed:', error);
    return res.status(500).json({ error: 'Internal Server Error' });
  }
};

// Profile update endpoint
router.post('/update', verifyUser, async (req, res) => {
  try {
    const { educationLevel, programmingExperience, roboticsBackground } = req.body;

    // Update user profile using Better Auth API
    await auth.api.updateUser({
      body: {
        educationLevel,
        programmingExperience,
        roboticsBackground,
      },
      headers: fromNodeHeaders(req.headers)
    });

    // Fetch fresh session to get updated user data
    const session = await auth.api.getSession({
      headers: fromNodeHeaders(req.headers)
    });

    if (!session?.user) {
      return res.status(500).json({ error: 'Failed to get updated user' });
    }

    return res.json({ user: session.user });
  } catch (error) {
    console.error('Error updating profile:', error);
    return res.status(500).json({ error: 'Failed to update profile' });
  }
});

// Get user profile endpoint
router.get('/profile', verifyUser, async (req, res) => {
  try {
    // The user is already populated by verifyUser middleware
    const user = (req as any).user;

    if (!user) {
      return res.status(404).json({ error: 'User not found' });
    }

    return res.json({ user });
  } catch (error) {
    console.error('Error fetching profile:', error);
    return res.status(500).json({ error: 'Failed to fetch profile' });
  }
});

export default router;