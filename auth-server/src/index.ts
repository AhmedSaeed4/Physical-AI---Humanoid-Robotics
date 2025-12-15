import 'dotenv/config';
import express from 'express';
import cors from 'cors';
import { auth } from './auth';
import { toNodeHandler } from 'better-auth/node';
import authRoutes from './routes/auth';
import userRoutes from './routes/user';

const app = express();
const PORT = process.env.PORT || 3001;

// Middleware
// CORS must be first
app.use(cors({
  origin: [process.env.FRONTEND_URL || 'http://localhost:3000', process.env.BACKEND_URL || 'http://localhost:8000'],
  credentials: true,
  methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
  allowedHeaders: ["Content-Type", "Authorization"],
}));

// Better Auth handler (must be before body parsers)
app.all("/api/auth/*", toNodeHandler(auth));

// Body parsers
app.use(express.json());

// Routes
app.use('/api', authRoutes);
app.use('/api/user', userRoutes);

app.get('/', (req, res) => {
  res.json({ message: 'Auth Server is running!' });
});

app.listen(PORT, () => {
  console.log(`Auth Server running on port ${PORT}`);
});

export default app;