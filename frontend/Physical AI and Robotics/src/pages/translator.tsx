import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function Translator(): JSX.Element {
  return (
    <Layout
      title="Urdu Translator"
      description="Translate AI-Native Robotics content to Urdu">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>🌐 Urdu Translator</h1>
            <p className="hero__subtitle">
              Translation feature coming soon! This tool will help translate
              AI-Native Robotics content into Urdu.
            </p>
            <div className="margin-top--lg">
              <Link
                className="button button--primary button--lg"
                to="/docs/introduction">
                Browse Documentation
              </Link>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}
