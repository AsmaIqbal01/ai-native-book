import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function Login(): JSX.Element {
  return (
    <Layout
      title="Login"
      description="User authentication for AI-Native Robotics">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>🔐 Login</h1>
            <p className="hero__subtitle">
              User authentication is coming soon! This feature will enable
              personalized learning experiences.
            </p>
            <div className="margin-top--lg">
              <Link
                className="button button--primary button--lg"
                to="/">
                Return Home
              </Link>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}
