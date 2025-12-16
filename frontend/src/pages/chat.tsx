import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function ChatPage() {
  return (
    <Layout title="AI Chat" description="Chat with our AI assistant about the book content">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <h1>Book Content Q&A</h1>
            <p>Ask questions about the Physical AI & Humanoid Robotics book content</p>
            <BrowserOnly fallback={<div>Loading chat...</div>}>
              {() => {
                const ChatBot = require('../components/ChatBot').default;
                return <ChatBot />;
              }}
            </BrowserOnly>
          </div>
        </div>
      </div>
    </Layout>
  );
}