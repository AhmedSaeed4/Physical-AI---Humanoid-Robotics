import Layout from '@theme/Layout';
import ChatBot from '../components/ChatBot';

export default function ChatPage() {
  return (
    <Layout title="AI Chat" description="Chat with our AI assistant about the book content">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <h1>Book Content Q&A</h1>
            <p>Ask questions about the Physical AI & Humanoid Robotics book content</p>
            <ChatBot />
          </div>
        </div>
      </div>
    </Layout>
  );
}